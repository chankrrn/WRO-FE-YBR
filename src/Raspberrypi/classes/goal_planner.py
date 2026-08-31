"""
Goal-based planning: pick poses the robot must reach, then plan a drivable
path through them.

This replaces the offset-profile approach the final round used to drive, where
a pillar bent the racing line sideways by a computed number of millimetres and
pure pursuit followed the bent line. That worked until the corridor ran out of
room, at which point the offset was silently clamped to whatever fitted and
the robot drove past the pillar at whatever clearance was left over - which on
a 1000mm corridor with a 525mm demand was, for a pillar on the same side as
its own dodge, none at all. The failure was invisible: nothing in the loop
knew the difference between "planned to pass at 300mm" and "asked for 525mm,
got 25mm, drove it anyway".

The model here is the ordinary one:

    GOAL      a pose - x, y AND heading - that the robot has to reach.
              Beside each pillar, on the side its colour dictates, squared up
              with the track. Plus plain route goals round the rest of the lap
              so there is always something ahead to drive at.

    PATH      the shortest curvature-bounded path through those goals in
              order, chained pose to pose with Dubins (classes/dubins.py).
              Curvature-bounded means every arc in it is one the steering can
              actually hold - a goal the robot cannot reach is rejected here,
              not discovered halfway through the corner.

    CHECK     the path is swept with the robot's own footprint against the
              pillars, the walls, the centre block and the parking bay. A
              path that collides is not driven; the goal that produced it is
              retried further out, and if nothing fits the plan says so.

    FOLLOW    pure pursuit chases a point one lookahead along the result,
              exactly as it chases the racing line in qualification. The
              follower did not need changing and has not been changed.

The property that matters, and the reason for the rewrite: a goal that cannot
be met is REPORTED. `Plan.compromised` and `Plan.reason` say which pillar could
not be given its clearance and how much it actually got, so a pass that is
going to be tight shows up in the status line on the approach instead of as a
collision nobody can explain afterwards.

    planner = GoalPlanner(field_map, racing_line, min_radius_mm=250)
    plan = planner.plan(pose, direction, pillars)
    target = plan.target_point(pose, lookahead_mm=300)

Everything is millimetres and the field frame from field_map.py; headings are
degrees clockwise from +Y.
"""
import math
import time
from collections import namedtuple

import numpy as np

from classes.dubins import dubins_cost, plan_dubins

# A pillar as the planner wants it: where it is and which way it must be
# passed. Deliberately not BlockMap's own type - the planner should not care
# how a pillar came to be believed in, only where it is.
Obstacle = namedtuple("Obstacle", "x y side")

# One goal pose in the chain, and why it exists. `obstacle` is None for the
# plain route goals that carry the plan round the rest of the lap.
Goal = namedtuple("Goal", "x y heading obstacle clearance_mm progress offset "
                          "fallbacks")

# A goal before it becomes a pose: where along the lap it sits, how far to the
# right of the racing line, and how hard it is to argue with. `rank` is what
# _thin consults when two nodes are too close together to both be reached.
_Node = namedtuple("_Node", "progress offset rank obstacle clearance fallbacks")
_ROUTE = 0      # keeps the plan near the racing line - always expendable
_GATE = 1       # a pillar's pass or exit - the reason the plan exists
_ROBOT = 2      # where the robot actually is - not negotiable

# How much more room than the bare S-curve geometry needs is demanded before
# two nodes are accepted as reachable. At exactly 1.0 the connecting path is
# two arcs at full lock with no straight between them and nothing in hand for
# tracking error, which the follower cannot hold; the margin buys the slack
# back. Raise it if the plan still asks for more lock than the robot has.
SPAN_MARGIN = 1.35

# A path that turns through more than this in total is doing a loop: Dubins
# answers an unreachable goal with the shortest path that exists, and where
# the goal is too close or too far off to the side, the shortest thing that
# exists is a circle. Nothing legitimate on this track turns further - the
# sharpest thing the plan ever contains is a 90-degree corner and a pass
# either side of it - so this is a clean test for "that goal cannot be met
# from here". See _chain.
LOOP_TURN_DEG = 270.0

# How many progressively tighter passes a pillar's goal is retried at when the
# one it wanted turns out to be unreachable from where the robot is. They run
# from the offset asked for down to the tightest that still clears the block at
# all - never past it and never across it, so a relaxed pass is always still on
# the side the colour requires. Giving up some of the dodge is bad; driving a
# circle in front of the pillar is worse, and driving into it is worse still.
RELAXATION_STEPS = 4

# How far a pass goal's heading may be turned off the track heading, in
# degrees, when arriving square to the track cannot be reached from where the
# robot is.
#
# A goal beside a pillar says two things: BE HERE, and be FACING THIS WAY.
# The first is what clears the pillar; the second is only a preference, and
# insisting on it is what made pass goals unreachable near a corner - where
# the track heading is swinging, the robot is close, and there is no
# curvature-bounded path that arrives both beside the pillar and square to a
# line that is itself turning. The planner used to give up on those and drop
# the goal, which left only the exit goal past the pillar, which let the path
# bend smoothly around to it THROUGH the pillar. Being 25 degrees off square
# beside a pillar is not a problem; not being beside it is.
#
# Tried in this order, so square is still preferred wherever it fits. Both
# signs, because which way helps depends on the corner and the side.
PASS_HEADINGS_DEG = (0.0, 12.0, -12.0, 25.0, -25.0, 40.0, -40.0)

# ============================================================================
# Geometry
# ============================================================================
# The pillars are 50mm square (block_map.BLOCK_SIZE_MM), taken as a disc of
# this radius for collision purposes. The circumscribed radius rather than the
# inscribed one, because the block's yaw is not tracked and cannot be - a
# square looks the same from every side - so the only honest bound is the one
# that holds at every orientation.
BLOCK_RADIUS_MM = 50.0 * math.sqrt(2.0) / 2.0

# How finely a candidate path is swept for collisions. Half the robot's own
# width would be the loose bound; this is well inside it, and the sweep is one
# vectorised numpy expression whatever the count.
SWEEP_STEP_MM = 25.0

# The robot is checked as three discs down its centerline - rear axle, middle,
# nose - rather than one. A single disc at the centre is optimistic by the
# whole overhang, which is exactly the part that swings out in a corner and
# exactly the part that clips a pillar on the way past.
FOOTPRINT_STATIONS = (-1.0, 0.0, 1.0)


def _turn_total_deg(headings):
    """
    Total heading swept along a SAMPLED path, unwrapped, in degrees.

    The planner itself does not use this - it screens candidates with
    dubins_cost, which gets the same number analytically and without drawing
    the path. This is the independent version, measured off the finished
    geometry, and it exists for test_goal_planner.py: a loop check that shares
    its arithmetic with the thing it is checking is not a check.

    Unwrapped because the raw headings wrap at 360 and a naive difference
    would read a loop as a series of small steps with one enormous one in it.
    """
    if len(headings) < 2:
        return 0.0
    return float(np.sum(np.abs(np.diff(np.unwrap(headings, period=360.0)))))


class Plan:
    """
    A planned path plus the reasoning behind it.

    `points` is Nx2 in field mm and `headings` is N in degrees; together they
    are what pure pursuit follows. The rest is for the status line and the
    debug overlay, and for answering "why did it do that" after a run.
    """

    def __init__(self, points, headings, goals, compromised=False, reason="",
                 planned_at=None):
        self.points = points
        self.headings = headings
        self.goals = goals
        self.compromised = bool(compromised)
        self.reason = reason
        # The measured worst gaps along the whole plan, filled in by
        # GoalPlanner.check(). Declared here rather than only there so that a
        # Plan is never half-built: everything that reads them - the status
        # line, the debug overlay, the tests - can read them off any Plan.
        # inf means "nothing of that kind was in range", not "not measured".
        self.pillar_gap_mm = float("inf")
        self.wall_gap_mm = float("inf")
        self.planned_at = time.monotonic() if planned_at is None else planned_at
        # Cumulative arc length, so target_point can step a lookahead along
        # the path without re-measuring it every tick.
        if len(points) > 1:
            steps = np.hypot(*np.diff(points, axis=0).T)
            self.arc = np.concatenate(([0.0], np.cumsum(steps)))
        else:
            self.arc = np.zeros(len(points))

    def __len__(self):
        return len(self.points)

    @property
    def length_mm(self):
        return float(self.arc[-1]) if len(self.arc) else 0.0

    # ========================================================================
    # FOLLOWING
    # ========================================================================

    def nearest_index(self, x, y):
        """Index of the closest sampled point to a field position."""
        offsets = self.points - np.array([x, y])
        return int(np.argmin(np.einsum("ij,ij->i", offsets, offsets)))

    def cross_track_mm(self, x, y):
        """How far the robot is off the plan, unsigned."""
        index = self.nearest_index(x, y)
        return float(np.hypot(*(self.points[index] - np.array([x, y]))))

    def target_point(self, pose, lookahead_mm):
        """
        The point one lookahead along the plan from the robot's closest point
        on it - what pure pursuit steers at.

        Clamped to the last point rather than wrapping: the plan is a finite
        stretch ahead, not a loop, and running off the end of it is the
        planner's cue to make a new one, not something to paper over by
        aiming back at the start.
        """
        if len(self.points) == 0:
            return None
        index = self.nearest_index(pose.x, pose.y)
        wanted = self.arc[index] + float(lookahead_mm)
        ahead = int(np.searchsorted(self.arc, wanted))
        if ahead >= len(self.points):
            return tuple(self.points[-1])
        return tuple(self.points[ahead])

    def remaining_mm(self, x, y):
        """Path left in front of the robot - the trigger to replan."""
        return self.length_mm - float(self.arc[self.nearest_index(x, y)])

    # ========================================================================
    # DRAWING
    # ========================================================================

    def draw(self, canvas, to_px, color=(90, 200, 220), thickness=2):
        """Draws the plan and its goals onto a top-down canvas."""
        import cv2

        if len(self.points) > 1:
            pixels = np.array([to_px(x, y) for x, y in self.points], dtype=np.int32)
            cv2.polylines(canvas, [pixels], isClosed=False, color=color,
                          thickness=thickness)
        for goal in self.goals:
            spot = to_px(goal.x, goal.y)
            # A goal beside a pillar is the one worth seeing; the route goals
            # that carry the plan round the lap are drawn smaller.
            if goal.obstacle is None:
                cv2.circle(canvas, spot, 3, (120, 120, 120), 1)
                continue
            cv2.circle(canvas, spot, 6, (60, 220, 240), 2)
            radians = math.radians(goal.heading)
            nose = to_px(goal.x + 90.0 * math.sin(radians),
                         goal.y + 90.0 * math.cos(radians))
            cv2.line(canvas, spot, nose, (60, 220, 240), 2)
        return canvas


class GoalPlanner:
    """
    Turns "where are the pillars" into "here is a path I have checked".

    Stateless per call: plan() takes the pose and the pillars and returns a
    Plan. What to do when there is no acceptable plan, and how often to ask
    for a new one, are the caller's decisions - see FinalTask.
    """

    def __init__(self, field_map, racing_line, min_radius_mm,
                 robot_half_width_mm=70.0, robot_front_mm=120.0,
                 robot_rear_mm=90.0, clearance_mm=150.0,
                 min_clearance_mm=45.0, wall_clearance_mm=40.0,
                 horizon_mm=2200.0, route_spacing_mm=550.0, max_gates=3,
                 approach_mm=450.0, exit_mm=350.0, step_mm=SWEEP_STEP_MM):
        """
        I/O:
            min_radius_mm: tightest arc a plan may contain. Pass the robot's
                           turning circle with a margin - a path at exactly
                           full lock leaves nothing to correct with.
            clearance_mm: the gap the planner ASKS for between the robot's
                          body and a pillar. Unlike the old clearance_mm this
                          is a preference, not a promise: where the corridor
                          cannot give it, the plan takes less and says so.
            min_clearance_mm: the gap below which a pass is not worth calling
                              a pass. A plan that cannot beat this is
                              compromised and the caller should slow down.
            horizon_mm: how much lap the plan covers. Must comfortably exceed
                        the pursuit lookahead or the robot drives off the end
                        of its own plan between replans.
            max_gates: how many pillars one plan reaches through. Beyond two
                       or three the far ones are re-planned before they are
                       reached anyway, and each costs a pair of goals.
            approach_mm: how far short of a pillar the route goals stop, so
                         nothing pulls the robot back onto the centerline on
                         the run-in to a pass.
            exit_mm: how far past a pillar the plan holds its offset, so the
                     tail is clear before the line folds back.
        """
        self.map = field_map
        self.path = racing_line
        self.min_radius_mm = float(min_radius_mm)
        self.half_width_mm = float(robot_half_width_mm)
        self.front_mm = float(robot_front_mm)
        self.rear_mm = float(robot_rear_mm)
        self.clearance_mm = float(clearance_mm)
        self.min_clearance_mm = float(min_clearance_mm)
        self.wall_clearance_mm = float(wall_clearance_mm)
        self.horizon_mm = float(horizon_mm)
        self.route_spacing_mm = float(route_spacing_mm)
        self.max_gates = int(max_gates)
        self.approach_mm = float(approach_mm)
        self.exit_mm = float(exit_mm)
        self.step_mm = float(step_mm)

    # ========================================================================
    # PLANNING
    # ========================================================================

    def plan(self, pose, direction, obstacles=()):
        """
        A checked path from the robot's pose, through the pillars ahead, out
        to the horizon.

        I/O:
            pose: current Pose (x, y, heading)
            direction: +1 or -1, which way round the loop - see RacingLine
            obstacles: iterable of Obstacle, in field mm. Pillars behind the
                       robot are ignored here rather than by the caller, so
                       a pillar just driven past cannot pull the plan back.
            return: Plan. Never None - a plan that could not clear everything
                    is still returned, with `compromised` set and `reason`
                    saying which pillar and by how much, because the robot
                    has to drive something and a bad plan it knows about
                    beats no plan at all.
        """
        progress, lateral = self.path.project(pose.x, pose.y, direction)
        ahead = self._obstacles_ahead(progress, direction, obstacles)
        goals, compromised, reason = self._build_goals(progress, lateral,
                                                       direction, ahead)
        points, headings, driven, skipped = self._chain(pose, goals, direction)
        if skipped:
            compromised = True
            late = "; ".join(f"pillar {goal.progress - progress:+.0f}mm ahead "
                             f"unreachable - seen too late"
                             for goal in skipped)
            reason = "; ".join(filter(None, [reason, late]))

        # The pass a goal was RELAXED to is the one that will be driven, so it
        # is the one the plan has to answer for - re-read the shortfalls off
        # the goals that survived rather than off the ones that were asked for.
        for goal in driven:
            if goal.obstacle is not None and goal.clearance_mm < self.min_clearance_mm:
                compromised = True
                shortfall = (f"pillar {goal.progress - progress:+.0f}mm ahead "
                             f"gets only {goal.clearance_mm:.0f}mm "
                             f"(asked {self.clearance_mm:.0f})")
                if shortfall not in reason:
                    reason = "; ".join(filter(None, [reason, shortfall]))
        plan = Plan(points, headings, driven, compromised, reason)
        return self.check(plan, [item[3] for item in ahead])

    def _obstacles_ahead(self, progress, direction, obstacles):
        """
        The pillars in front of the robot, nearest first, with the progress
        each sits at attached.

        "In front" is measured along the lap rather than off the nose, so a
        pillar around the upcoming corner still counts and one just passed
        does not - which is the whole reason the plan is built in the path
        frame instead of the robot's.
        """
        found = []
        for obstacle in obstacles:
            at, lateral = self.path.project(obstacle.x, obstacle.y, direction)
            gap = self.path.gap(progress, at)
            # A small negative gap is a pillar the robot is still alongside;
            # it must keep its goal or the plan snaps straight into it. One
            # robot length back is enough for the tail to have cleared.
            if gap < -(self.rear_mm + BLOCK_RADIUS_MM) or gap > self.horizon_mm:
                continue
            found.append((gap, at, lateral, obstacle))
        found.sort(key=lambda item: item[0])
        return found[:self.max_gates]

    def _build_goals(self, progress, lateral, direction, ahead):
        """
        The goal chain: two beside each pillar, plus route goals filling the
        empty stretches out to the horizon.

        Each pillar gets a PASS goal at its own progress and an EXIT goal one
        exit_mm further on, both at the same offset and both squared up with
        the track - the heading is the racing line's, so the robot arrives
        beside the pillar pointing down the lap rather than across it.

        The exit goal is what keeps the robot out until its tail is clear.
        Without it the plan is free to start folding back toward the line the
        instant the CENTRE of the robot draws level with the pillar, which is
        the moment the rear overhang is still swinging past it. Naming a pose
        the robot must still be in AFTER the pillar costs one goal and removes
        the whole class of clipped-on-the-way-out failures.

        Aiming at a point beside a pillar without also saying which way to be
        facing there is what lets a follower arrive correctly placed and still
        turned into it - which is the argument for goal POSES over goal points,
        and the reason every goal here carries a heading.

        Built as nodes first and turned into poses at the end, because the two
        things that make a chain undrivable - a goal too close to the one
        before it for the offset between them, and an offset that tightens a
        corner past the turning circle - are both properties of NEIGHBOURING
        nodes, and are far easier to enforce on a list of (progress, offset)
        pairs than on finished poses. See _thin.
        """
        nodes = [_Node(progress, lateral, _ROBOT, None, float("inf"), ())]
        horizon = progress + self.horizon_mm
        walked, compromised, reasons = progress, False, []

        for _, at, pillar_lateral, obstacle in ahead:
            offset, achieved = self._gate_offset(at, pillar_lateral, obstacle,
                                                 direction)
            exit_at = at + self.exit_mm
            # Route nodes bridging the empty stretch since the last pillar, so
            # a long run does not become one enormous Dubins segment that
            # ignores the racing line and cuts the corner off. They stop an
            # approach_mm short of the pillar: a route node sitting on the
            # centerline just before a pass would pull the robot back onto the
            # line at exactly the wrong moment.
            nodes.extend(self._route_nodes(walked, at - self.approach_mm))
            fallbacks = self._fallback_offsets(offset, pillar_lateral,
                                               obstacle.side)
            nodes.append(_Node(at, offset, _GATE, obstacle, achieved, fallbacks))
            # The exit node carries the SAME offset as the pass, not one
            # re-derived at its own progress. Re-deriving it looks more
            # careful and measures worse: the corridor and corner limits move
            # between the two, so the exit can come out wider than the pass it
            # is supposed to be settling out of, and the plan bulges after the
            # pillar instead of easing back. Measured over 24 runs that turned
            # 0-1 runs with wall contact into 5-8.
            nodes.append(_Node(exit_at, offset, _GATE, obstacle, achieved,
                               fallbacks))
            if achieved < self.min_clearance_mm:
                compromised = True
                reasons.append(f"pillar {at - progress:+.0f}mm ahead gets only "
                               f"{achieved:.0f}mm (asked {self.clearance_mm:.0f})")
            walked = exit_at

        # The tail, out to the horizon - or past it, where a pillar's exit
        # node already ran beyond. Without this floor the chain can END on a
        # pass, which leaves the robot following a plan that stops beside a
        # pillar: the follower clamps to the last point, stops steering, and
        # the round quietly falls apart a metre later. A plan must always
        # reach further ahead than the lookahead can look.
        nodes.extend(self._route_nodes(
            walked, max(horizon, walked + self.route_spacing_mm)))

        goals = []
        for node in self._thin(nodes)[1:]:      # [0] is the robot itself
            goals.append(self._goal_at(node.progress, node.offset, direction,
                                       node.obstacle, node.clearance,
                                       node.fallbacks))
        return goals, compromised, "; ".join(reasons)

    def _goal_at(self, progress, offset, direction, obstacle=None,
                 clearance=float("inf"), fallbacks=(), heading_deg=0.0):
        """
        A goal pose on the racing line at `progress`, held `offset` to the
        right of it and squared up with the track - or turned `heading_deg`
        off square, where square cannot be reached (see PASS_HEADINGS_DEG).
        """
        x, y = self.path.point_at(progress, direction, offset)
        _, _, heading = self.path.pose_at(progress, direction)
        return Goal(x, y, (heading + heading_deg) % 360.0, obstacle, clearance,
                    progress, offset, tuple(fallbacks))

    def _fallback_offsets(self, offset, pillar_lateral, side):
        """
        Tighter passes to fall back on, from the one asked for down to the
        tightest that still clears the block at all.

        Interpolated between two offsets that are BOTH on the required side of
        the pillar, rather than scaled toward zero. Scaling toward zero looks
        equivalent and is not: for a pillar already sitting off the racing
        line, a scaled-down offset crosses the pillar and ends up passing it on
        the side the rules forbid - trading a tight legal pass for a roomy
        illegal one, which scores worse than the collision it was avoiding.

        I/O:
            return: tuple of (offset_mm, clearance_mm), widest first, not
                    including the offset already asked for
        """
        floor = pillar_lateral + side * (BLOCK_RADIUS_MM + self.half_width_mm)
        # Already at or inside the floor: the corridor or the corner has
        # clamped this pass tighter than a fallback would, so there is nothing
        # to give up. Returning steps here would offer offsets WIDER than the
        # one being tried, which reads as a relaxation and is the opposite.
        if abs(offset - pillar_lateral) <= abs(floor - pillar_lateral):
            return ()
        steps = []
        for index in range(1, RELAXATION_STEPS):
            fraction = index / float(RELAXATION_STEPS)
            tighter = offset + (floor - offset) * fraction
            steps.append((tighter, abs(tighter - pillar_lateral)
                          - BLOCK_RADIUS_MM - self.half_width_mm))
        return tuple(steps)

    def _route_nodes(self, from_progress, to_progress):
        """
        Plain nodes on the racing line, spaced along an empty stretch.

        Without these a Dubins segment spanning a whole side of the field is
        free to bow out to the wall on its way - it is the shortest path
        between two poses, and nothing in it prefers the racing line. Pinning
        it every route_spacing_mm keeps the plan on the line wherever there is
        no reason to leave it, which is most of the lap.
        """
        span = self.path.gap(from_progress, to_progress)
        if span <= 0.0:
            return []
        count = max(1, int(round(span / self.route_spacing_mm)))
        return [_Node(from_progress + span * step / count, 0.0, _ROUTE, None,
                      float("inf"), ())
                for step in range(1, count + 1)]

    def _reachable_span_mm(self, offset_change):
        """
        The least along-track distance in which the plan can move sideways by
        `offset_change` and still be drivable.

        Two goals with the same heading and a lateral step between them are
        joined by an S of two arcs at the turning circle. That S needs room:
        for a step d between poses at radius R the arcs turn through
        acos(1 - d/2R) each, and the pair spans 2R sin of that. Ask for the
        step in less distance than that and there IS no curvature-bounded path
        with those end headings - Dubins will answer with a loop, because a
        loop is the shortest thing that exists, and the robot will drive a
        circle in the middle of the track.

        That loop is not hypothetical: it is what the first version of this
        planner did, and it is why the spacing is enforced here rather than
        left to the solver to complain about. Dubins never complains.

        I/O:
            return: mm, or inf if the step is more than the geometry can do
                    at any distance (a step wider than two turning circles)
        """
        step = abs(offset_change)
        if step < 1e-6:
            return 0.0
        radius = self.min_radius_mm
        if step >= 2.0 * radius:
            return float("inf")
        return 2.0 * radius * math.sin(math.acos(1.0 - step / (2.0 * radius)))

    def _thin(self, nodes):
        """
        Drops nodes the chain cannot reach in the room available.

        Walks the list keeping a running "last node that survived", and where
        the next one is closer than _reachable_span_mm allows, removes the
        LESS important of the two: a route node exists only to keep the plan
        near the racing line and is always worth losing, while a pillar's pass
        and exit nodes are the reason the plan exists and are never dropped.
        The robot's own node heads the list and outranks everything, since the
        chain has to start where the robot actually is.

        The usual outcome on the approach to a pillar is that the last route
        node before it disappears, and the plan runs from wherever the robot
        is straight into the pass - which is the correct answer, and the one
        the old profile could not express because it had to be continuous
        everywhere.
        """
        kept = [nodes[0]]
        for node in nodes[1:]:
            while True:
                previous = kept[-1]
                span = self.path.gap(previous.progress, node.progress)
                needed = (self._reachable_span_mm(node.offset - previous.offset)
                          * SPAN_MARGIN)
                if span >= needed:
                    kept.append(node)
                    break
                if previous.rank == _ROUTE:
                    kept.pop()          # the earlier node was the expendable one
                    continue
                if node.rank == _ROUTE:
                    break               # keep what we have, drop this node
                # Two nodes that both have to exist, too close together for the
                # step between them - a pillar the corridor wants passed on one
                # side hard on the heels of one that wanted the other. Both are
                # kept and _chain relaxes the offsets until something is
                # drivable, because the alternative is dropping a pillar's goal
                # and driving at it as though it were not there.
                kept.append(node)
                break
        return kept

    def _lateral_bounds(self, direction):
        """
        How far the plan may sit either side of the racing line, as
        (low, high) in the travel frame where + is to the right of travel.

        Which SIGN points at the outer wall depends on which way round the
        loop is being driven - the wall is on the right going the +1 way and
        on the left going the other - so a symmetric clamp would restrict the
        centre-block side by mistake half the time.
        """
        toward_wall, toward_block = self.path.lateral_room_mm(
            self.wall_clearance_mm + self.half_width_mm)
        if direction > 0:
            return -toward_block, toward_wall
        return -toward_wall, toward_block

    def _gate_offset(self, at, lateral, obstacle, direction):
        """
        Where beside a pillar the robot's centerline should pass, and the body
        clearance that actually buys.

        The wanted offset is the pillar's own lateral position plus, on the
        side its colour dictates, enough room for the block's half-diagonal,
        the robot's half width and the clearance asked for. Where the corridor
        will not take that, the offset is pulled back to the wall limit and
        THE CLEARANCE THAT SURVIVES IS RETURNED rather than assumed.

        That return value is the whole point of the rewrite. The old code did
        the same clamp and then carried on as though the clearance were
        intact, so a pillar the corridor could not accommodate produced a
        confident plan and a collision. Here the shortfall comes back with the
        offset, reaches Plan.reason, and reaches the status line.

        I/O:
            return: (offset_mm in the travel frame, achieved body clearance)
        """
        needed = BLOCK_RADIUS_MM + self.half_width_mm + self.clearance_mm
        wanted = lateral + obstacle.side * needed
        low, high = self._lateral_bounds(direction)

        # Inside a bend the corridor is not the only limit. A plan held `d` mm
        # inside an arc of radius R is itself an arc of radius R - d, so an
        # inward pass spends the corner's radius one for one, and at
        # R - d below the turning circle there is no path that follows it.
        # Dubins answers that with a loop rather than an error (see
        # _reachable_span_mm), so it has to be caught rather than discovered.
        #
        # This limit was tried as a property of the EXIT goal alone, on the
        # reasoning that a pass goal is only gone THROUGH and a path may touch
        # a curvature it could not sustain. It is true, and it does not help:
        # letting the pass reach further inside a bend bought nothing on
        # pillar clearance (97%/87% either way, measured over 24 runs) and
        # cost a great deal on the walls, because the path that cuts in at an
        # angle also bows inside the goal it is cutting to, and inside a bend
        # what it bows into is the centre block. Wall contact went from 0-1
        # runs in 12 to 5-8. The cap does two jobs and only one of them is
        # about curvature.
        #
        # Measured against the BARE turning circle: min_radius_mm already
        # carries the caller's margin (see FinalTask._steerable_radius_mm),
        # and padding it twice cost a corner pass most of its clearance.
        inward = self.path.inward_limit_mm(at, direction, self.min_radius_mm)
        if math.isfinite(inward):
            if self.path.inward_sign(direction) < 0.0:
                low = max(low, -inward)
            else:
                high = min(high, inward)

        offset = float(np.clip(wanted, low, high))

        # The clamp can land the centerline on the WRONG SIDE of the pillar,
        # which is not a tight pass but a pass on the side the rules forbid.
        # Where that happens the offset is pushed to the far edge of the
        # corridor on the required side instead - a pass that is legal and
        # tight beats one that is roomy and scored as a fault.
        if (offset - lateral) * obstacle.side <= 0.0:
            offset = high if obstacle.side > 0 else low
        return offset, abs(offset - lateral) - BLOCK_RADIUS_MM - self.half_width_mm

    # ========================================================================
    # CHAINING
    # ========================================================================

    def _chain(self, pose, goals, direction):
        """
        Dubins from the pose through every goal in order, concatenated.

        Each segment starts where the last one ended - at the goal pose, not
        at wherever the last sample happened to fall - so the joins are exact
        and the whole chain stays inside the curvature bound.

        Two things can go wrong with a segment, and they want opposite
        treatment:

          * it will not solve at all, which for Dubins means the two poses sat
            on top of each other. Skipped; the next goal picks the chain up.

          * it solves into a LOOP. Dubins returns the shortest path that
            exists, and when a goal is too close, or too far off to the side
            to reach in the room left, the shortest path that exists is a
            circle - so the failure arrives disguised as a valid answer.
            _thin removes most of these by construction, but it reasons about
            a straight reference and the real track bends, so the ones a
            corner creates survive it. They are caught here instead, on the
            finished geometry, where there is nothing left to be wrong about.

        A looping ROUTE goal is dropped - it only existed to keep the plan
        near the racing line. A looping PILLAR goal is retried at progressively
        smaller offsets (RELAXATIONS) until one is reachable, and the clearance
        that survives is written back into the goal, so a pass the robot had to
        give ground on still reports what it actually got.

        I/O:
            return: (points Nx2, headings N, goals actually used)
        """
        points = [np.array([[pose.x, pose.y]])]
        headings = [np.array([pose.heading])]
        cursor = (pose.x, pose.y, pose.heading)
        driven, skipped = [], []

        for goal in goals:
            # Screen every candidate analytically first - dubins_cost does not
            # sample the path - and draw only the one that wins. With seven
            # headings against four offsets that is 28 candidates a pillar, so
            # the difference between costing and drawing them is the
            # difference between fitting in a control tick and not.
            attempt = None
            for offset, clearance, turn in self._attempts(goal):
                candidate = self._goal_at(goal.progress, offset, direction,
                                          goal.obstacle, clearance,
                                          goal.fallbacks, heading_deg=turn)
                _, turned = dubins_cost(
                    cursor, (candidate.x, candidate.y, candidate.heading),
                    self.min_radius_mm)
                if turned <= LOOP_TURN_DEG:
                    attempt = candidate
                    break

            segment = None
            if attempt is not None:
                segment = plan_dubins(cursor,
                                      (attempt.x, attempt.y, attempt.heading),
                                      self.min_radius_mm, step_mm=self.step_mm)
                if segment is not None and len(segment.points) < 2:
                    segment = None

            if segment is None:
                # Nothing from the full offset down to the tightest fallback
                # was reachable from here. A route goal is no loss. A PILLAR
                # goal is: skipping it drives the racing line straight through
                # the pass, which is the one outcome this planner exists to
                # prevent, so it is recorded and surfaced rather than dropped
                # quietly. It happens when a pillar is confirmed so late that
                # there is no longer room to get beside it - the exit goal
                # that follows is further away and usually still reachable, so
                # the pass often still happens, just late and tight.
                if goal.obstacle is not None:
                    skipped.append(goal)
                continue
            # Drop the first sample: it is the previous segment's last point.
            points.append(segment.points[1:])
            headings.append(segment.headings[1:])
            cursor = (attempt.x, attempt.y, attempt.heading)
            driven.append(attempt)

        return np.vstack(points), np.concatenate(headings), driven, skipped

    def _attempts(self, goal):
        """
        The (offset, clearance, heading turn) triples to try for one goal, best
        first.

        Ordered offset-outermost so that GIVING UP CLEARANCE IS THE LAST
        RESORT: every heading is tried at the full offset before any of them is
        tried at a reduced one. Clearance is what actually stops the robot
        touching the pillar; being square to the track is a preference, so the
        preference is what gets spent first.

        A route goal has neither to spend - it is one pose on the racing line,
        take it or leave it - so it is a single attempt.
        """
        if goal.obstacle is None:
            return [(goal.offset, goal.clearance_mm, 0.0)]
        triples = []
        for offset, clearance in [(goal.offset, goal.clearance_mm)] + list(goal.fallbacks):
            for turn in PASS_HEADINGS_DEG:
                triples.append((offset, clearance, turn))
        return triples

    # ========================================================================
    # CHECKING
    # ========================================================================

    def footprint(self, points, headings):
        """
        The robot's body swept along a path, as discs.

        Three per sample - rear axle, middle, nose - because a single disc at
        the centre understates the swing of the overhangs through a corner by
        exactly the amount that clips a pillar.

        I/O:
            return: Mx2 array of disc centres, M = 3 x len(points)
        """
        radians = np.radians(headings)
        forward = np.column_stack((np.sin(radians), np.cos(radians)))
        spans = {-1.0: -self.rear_mm, 0.0: 0.0, 1.0: self.front_mm}
        return np.vstack([points + forward * spans[station]
                          for station in FOOTPRINT_STATIONS])

    def clearances(self, points, headings, obstacles):
        """
        The worst gap between the robot's body and each hazard along a path.

        One vectorised pass over every disc against every hazard, which is
        what makes it affordable to check a plan properly rather than trusting
        the geometry that produced it.

        I/O:
            return: (pillar_gap_mm, wall_gap_mm). Either may be negative,
                    which means the body overlaps - a collision, not a
                    tight pass.
        """
        body = self.footprint(points, headings)

        pillar_gap = float("inf")
        for obstacle in obstacles:
            offsets = body - np.array([obstacle.x, obstacle.y])
            distance = np.min(np.hypot(offsets[:, 0], offsets[:, 1]))
            pillar_gap = min(pillar_gap,
                             distance - BLOCK_RADIUS_MM - self.half_width_mm)

        # Outer wall: how far the nearest disc centre stays inside the box,
        # less the body radius. The field is axis aligned, so the gap to the
        # box is the smaller of the two axes' gaps.
        to_outer = np.minimum(self.map.outer - np.abs(body[:, 0]),
                              self.map.outer - np.abs(body[:, 1]))
        # Centre block: OUTSIDE this box, so the gap is how far the nearest
        # disc is from it - positive on the larger of the two axes.
        to_inner = np.maximum(np.abs(body[:, 0]) - self.map.inner,
                              np.abs(body[:, 1]) - self.map.inner)
        wall_gap = float(min(np.min(to_outer), np.min(to_inner))) - self.half_width_mm

        for low, high in getattr(self.map, "obstacles", ()):
            # A bay wall is a box the body must stay out of: the gap is the
            # distance from each disc centre to the box, zero inside it.
            dx = np.maximum(np.maximum(low[0] - body[:, 0], body[:, 0] - high[0]), 0.0)
            dy = np.maximum(np.maximum(low[1] - body[:, 1], body[:, 1] - high[1]), 0.0)
            wall_gap = min(wall_gap,
                           float(np.min(np.hypot(dx, dy))) - self.half_width_mm)

        return pillar_gap, wall_gap

    def check(self, plan, obstacles):
        """
        Sweeps a finished plan and folds the result back into it.

        Called after planning rather than during: the goals are placed with
        the corridor already in mind, so this is the independent check that
        they were placed correctly, not the mechanism that places them. A plan
        that fails here is a bug in the goal placement or a pillar the
        corridor genuinely cannot accommodate - both worth knowing about.

        I/O:
            return: the same Plan, with `compromised` and `reason` updated
        """
        if len(plan.points) < 2:
            return plan
        pillar_gap, wall_gap = self.clearances(plan.points, plan.headings,
                                               obstacles)
        plan.pillar_gap_mm = pillar_gap
        plan.wall_gap_mm = wall_gap

        faults = []
        if pillar_gap < self.min_clearance_mm:
            faults.append(f"pillar gap {pillar_gap:.0f}mm")
        # Half of what was asked for, not zero. The goals are placed with a
        # full wall_clearance_mm in hand, but the path BETWEEN two goals is
        # free to bow a little further out than either of them, so measuring
        # against zero passes a plan that grazes the wall with nothing left
        # for pose error - and the pose is exactly what is least trustworthy
        # near a wall. Half is the point at which the bow has eaten enough of
        # the margin to be worth saying out loud.
        if wall_gap < self.wall_clearance_mm / 2.0:
            faults.append(f"wall gap {wall_gap:.0f}mm")
        if faults:
            plan.compromised = True
            plan.reason = "; ".join(filter(None, [plan.reason] + faults))
        return plan

    def __str__(self):
        return (f"GoalPlanner min_r={self.min_radius_mm:.0f}mm "
                f"clearance={self.clearance_mm:.0f}mm "
                f"horizon={self.horizon_mm:.0f}mm gates={self.max_gates}")

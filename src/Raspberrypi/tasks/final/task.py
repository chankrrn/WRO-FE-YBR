"""
Obstacle round: the same three laps, but each pillar has to be passed on the
side its colour dictates.

    GREEN -> the robot passes on the block's LEFT
    RED   -> the robot passes on the block's RIGHT

This round used to work by bending the racing line sideways: a pillar produced
a lateral offset, the offset was eased in and out with Beziers, and pure
pursuit followed the bent line. The idea was sound and the arithmetic was
careful, but the model had a hole in it. The offset the geometry asked for was
routinely more than the corridor could give - 525mm of demand into 1000mm of
corridor - so it was clamped, and NOTHING DOWNSTREAM KNEW. A pass planned at
300mm and a pass clamped to 25mm produced the same confident line, the same
status output, and one collision.

So the round now plans the ordinary way instead: it names GOAL POSES the robot
has to reach - beside each pillar, on the correct side, squared up with the
track - and asks GoalPlanner for a curvature-bounded path through them, which
it then sweeps with the robot's own footprint before driving it. Everything
about that lives in classes/goal_planner.py and classes/dubins.py; this file
supplies the goals, decides when to re-plan, and slows down when the planner
says a pass is going to be tight.

What did NOT change: lap counting, the speed profile, the lidar backstop, the
parking manoeuvre, and pure pursuit itself. The follower was never the
problem - it was being handed a line that had quietly stopped meaning what it
said.

One thing this round has to do before any of that is get OUT of the parking
space it starts in. The robot is placed between two walls that stick 200mm
out from the outer wall and are not in the map the particle filter matches
against, so the pose on tick one is the least trustworthy of the whole round.
Leaving is therefore driven open-loop off the lidar rather than off the pose -
see UnparkController - and the direction the lap runs is taken from which side
the lidar finds the outer wall on, not from the racing line's guess. See
_setup_manoeuvres and _lap_direction.
"""
import math
import time
from collections import namedtuple

import cv2

from classes.block_map import BLOCK_SIZE_MM
from classes.goal_planner import GoalPlanner, Obstacle
from classes.parking import (BayFinder, ParkingSequence, UnparkController,
                             nearest_outer_wall, section_of,
                             travel_direction_beside_wall, wall_heading_of,
                             wall_rects)
from classes.racing_line import RacingLine
from tasks.path_task import PathDrivingTask
from utils.angle_utils import angle_difference
from utils.enums import Color

# `lateral` is positive to the right of travel (see RacingLine.project).
# Passing a block on its LEFT means the robot ends up to the left of it, i.e.
# a negative offset from the block's own position. Measured in the TRAVEL
# frame, so it is the same number whichever way round the loop is being
# driven - which way that maps to on the mat is RacingLine's problem.
SIDE_FOR_COLOR = {Color.GREEN: -1.0, Color.RED: +1.0}

CAMERA_EVERY_N_TICKS = 2

# One pillar the plan is steering around: where it is, which way to pass it,
# and when it was last actually confirmed.
#
# The plan is built from THIS, not from nav.blocks.confirmed() directly. A
# plan rebuilt from live confirmations every tick does not degrade when a
# frame is missed, it is DELETED - the goal beside the pillar vanishes and the
# path snaps back to the racing line the moment BlockMap drops a track, which
# at range it does readily, because a pillar 2m off is a handful of pixels and
# MAX_MISSES is six frames. That flicker lands on the approach, where the
# pillar is furthest and the detection worst, and never on the exit, where it
# is close and solid.
Pillar = namedtuple("Pillar", "x y color last_seen")

# Two sightings closer together than this are the same pillar. Comfortably
# bigger than BlockMap's own ASSOCIATION_RADIUS_MM (180mm), since anything
# closer than this could not be dodged as a separate pillar anyway.
SAME_PILLAR_MM = 250.0

# How far behind the robot a pillar is forgotten. Measured from the robot's
# own progress rather than the pursuit target, and generous, because a pillar
# dropped while the tail is still going past it takes its goal with it and
# lets the plan fold back early - which is the one moment the exit goal exists
# to prevent. See GoalPlanner._build_goals.
FORGET_BEHIND_MM = 700.0

# How close to a parking space something has to be before it is not a pillar.
# The field never puts one within this of a bay, so anything the map has there
# is a bay wall read as a block. Waiting for such a "pillar" to be passed
# means waiting until the robot is past the BAY, which costs a whole lap - see
# _pillar_before_the_bay.
PILLAR_FREE_MM = 500.0


class FinalTask(PathDrivingTask):
    """
    PathDrivingTask with a goal-based planner in place of the racing line.

    Every tick the robot still projects itself onto the racing line for lap
    counting and speed, exactly as qualification does. What it STEERS at comes
    from GoalPlanner instead: a checked, curvature-bounded path through the
    goals beside the pillars ahead.

    With no pillars in sight the plan is the racing line, goal for goal, so
    the round degrades to qualification's behaviour rather than to something
    new and untested when the camera sees nothing.
    """

    name = "final"
    requires_camera = True

    def __init__(self, context, config=None, **kwargs):
        super().__init__(context, config=config, **kwargs)
        # Pillars being steered around, held across detection dropouts.
        self._pillars = []
        self.planner = None
        self._plan = None
        self._planned_pillars = 0
        self._replan_reason = "first plan"
        self._plans_made = 0
        # The parking bay: which section it is in, and the manoeuvre itself.
        self._start_section = None
        self._bay = None
        self._bay_progress = None
        self._bay_finder = None
        self._parking = None
        # Failed park attempts - see _retry_the_park. There is no cap and no
        # giving up: the round ends parked or on the time limit.
        self._park_attempts = 0
        # Millimetres still to reverse before the next attempt - see
        # _retry_the_park.
        self._park_backing_mm = 0.0
        # The exit from the bay the robot was placed in - see
        # _setup_manoeuvres.
        self._unparking = None
        # Where on the lap the exit handed back to the racing line, and
        # whether the reach warning has been said - see
        # _warn_if_parking_cannot_reach_the_bay.
        self._rejoin_progress = None
        self._warned_about_reach = False
        # Which way round the lap the bay says to go. None when the round is
        # not starting in one.
        self._bay_direction = None
        # The round starts INSIDE the bay, so this is where the bay is. Kept
        # as a raw point rather than a progress: the lap direction is not
        # settled yet, and project() needs it.
        self._start_point = None
    # ========================================================================
    # SETUP
    # ========================================================================

    def setup(self):
        super().setup()
        if self.context.object_solver is None:
            print("WARNING: no camera - the final round will drive the plain "
                  "racing line and ignore the pillars")
        self._apply_map_range()

        self.planner = GoalPlanner(
            self.context.nav.map, self.path,
            min_radius_mm=self._steerable_radius_mm(),
            robot_half_width_mm=float(self.setting("blocks.robot_half_width_mm")),
            robot_front_mm=float(self.setting("goals.robot_front_mm")),
            robot_rear_mm=float(self.setting("goals.robot_rear_mm")),
            clearance_mm=float(self.setting("blocks.clearance_mm")),
            min_clearance_mm=float(self.setting("goals.min_clearance_mm")),
            wall_clearance_mm=float(self.setting("blocks.wall_clearance_mm")),
            horizon_mm=float(self.setting("goals.horizon_mm")),
            route_spacing_mm=float(self.setting("goals.route_spacing_mm")),
            max_gates=int(self.setting("goals.max_gates")),
            approach_mm=float(self.setting("goals.approach_mm")),
            exit_mm=float(self.setting("goals.exit_mm")),
            align_mm=float(self.setting("goals.align_mm")))
        print(self.planner)

        pose = self.context.nav.get_pose()
        self._start_point = (pose.x, pose.y)
        self._start_section = section_of(pose.x, pose.y, self.context.nav.map)
        print(f"Parking bay expected in the {self._start_section} section")
        if self.setting("parking.enabled") and self._start_section is not None:
            self._bay_finder = BayFinder(
                self.context.nav.map, self._start_section,
                min_depth_mm=float(self.setting("parking.detect_min_depth_mm")),
                min_gap_mm=float(self.setting("parking.detect_min_gap_mm")),
                max_gap_mm=float(self.setting("parking.detect_max_gap_mm")),
                min_scans=int(self.setting("parking.detect_min_scans")),
                single_scans=int(self.setting("parking.detect_single_scans")))
        self._warn_if_the_corridor_is_too_narrow()
        self._warn_if_the_horizon_is_too_short()
        self._replan(pose, "setup")

    def _steerable_radius_mm(self):
        """
        The tightest arc the plan is allowed to contain: the robot's own
        turning circle with turn_radius_margin kept in hand.

        A path drawn at exactly full lock is one the robot can only just hold
        with nothing left to correct tracking error with, so every millimetre
        of error becomes a millimetre it can never recover. The margin is what
        the follower corrects inside.
        """
        return (self.pursuit.min_turn_radius_mm
                * float(self.setting("blocks.turn_radius_margin")))

    def _warn_if_the_corridor_is_too_narrow(self):
        """
        Says at setup whether the clearance being asked for is one the field
        can actually give, instead of leaving it to be discovered as a clamp.

        The corridor here is 1000mm wide and fixed. A pass wants the block's
        half-diagonal, the robot's half width and clearance_mm out of it, all
        measured from wherever the pillar happens to sit - so a clearance
        above about a third of the corridor cannot be met for any pillar that
        is not on the racing line, and above half of it cannot be met at all.
        The old default was 430mm into a 500mm half-corridor, which is why
        every pass was clamped and every clamp was silent.
        """
        clearance = float(self.setting("blocks.clearance_mm"))
        half = float(self.setting("blocks.robot_half_width_mm"))
        needed = BLOCK_SIZE_MM * math.sqrt(2.0) / 2.0 + half + clearance
        room = min(self.path.lateral_room_mm(
            float(self.setting("blocks.wall_clearance_mm")) + half))
        if needed > room:
            print(f"WARNING: blocks.clearance_mm={clearance:.0f} needs "
                  f"{needed:.0f}mm of offset but the corridor gives {room:.0f}mm - "
                  f"a pillar more than {room - needed + clearance:.0f}mm off the "
                  f"racing line cannot get it. Passes will be reported "
                  f"COMPROMISED; lower it to about "
                  f"{max(20.0, room - needed + clearance):.0f}.")
        else:
            print(f"Pillar passes ask {needed:.0f}mm of offset, corridor gives "
                  f"{room:.0f}mm")

    def _warn_if_the_horizon_is_too_short(self):
        """
        A plan the robot can outrun is a plan it drives off the end of, and a
        follower clamped to the last point of a stale plan stops steering.
        """
        reach = self.pursuit.lookahead_distance(float(self.setting("speed.base")))
        horizon = float(self.setting("goals.horizon_mm"))
        if horizon < reach * 3.0:
            print(f"WARNING: goals.horizon_mm={horizon:.0f} is short against a "
                  f"{reach:.0f}mm lookahead - raise it to at least "
                  f"{reach * 3.0:.0f} so the plan always reaches past where "
                  f"pure pursuit is aiming.")

    def _apply_map_range(self):
        """
        Lets this round push the range at which a pillar gets mapped at all.

        block_map's own MAX_MAPPING_RANGE_MM stays the default and the reason
        for it still holds: ObjectSolver ranges a pillar off its apparent
        height, so the error grows with the square of distance and a far
        pillar lands on the map in roughly the right direction but not the
        right place.

        Raising it matters less than it did. Under the old profile the offset
        was a SCHEDULE that had to start early, so mapping range was the real
        limit on how gently a dodge could begin. A goal is not a schedule - it
        is a pose, and the path to it is planned from wherever the robot
        actually is - so a pillar confirmed late gets a shorter, firmer
        approach rather than a step. What a long range still buys is warning;
        what it still costs is a red smudge on the wall getting a vote.
        """
        reach = self.setting("blocks.map_range_mm")
        if not reach:
            return
        blocks = self.context.nav.blocks
        print(f"Pillar mapping range: {blocks.max_range_mm:.0f}mm -> "
              f"{float(reach):.0f}mm (blocks.map_range_mm)")
        blocks.max_range_mm = float(reach)

    # ========================================================================
    # LEAVING THE BAY
    # ========================================================================

    def _setup_manoeuvres(self, pose):
        """
        Builds the bay exit, when the round is starting from inside a bay.

        Open loop and off the lidar rather than off the map: see
        UnparkController for why the pose is the wrong thing to trust on tick
        one of a run that starts in a slot.
        """
        if not self.setting("unpark.enabled"):
            return
        self._set_direction_from_the_bay(pose)
        self._unparking = UnparkController(
            lidar=self.context.lidar,
            reverse_mm=self.setting("unpark.reverse_mm"),
            reverse_steer_command=self.setting("unpark.reverse_steer_command"),
            steer_command=self.setting("unpark.steer_command"),
            forward_mm=self.setting("unpark.forward_mm"),
            speed=int(self.setting("unpark.speed")),
            reverse_speed=int(self.setting("unpark.reverse_speed")),
            look_s=float(self.setting("unpark.look_s")),
            servo_settle_s=float(self.setting("unpark.servo_settle_s")),
            side_bearing_deg=float(self.setting("unpark.side_bearing_deg")),
            side_sector_deg=float(self.setting("unpark.side_sector_deg")),
            side_margin_mm=float(self.setting("unpark.side_margin_mm")),
            in_bay_mm=float(self.setting("unpark.in_bay_mm")),
            default_side=int(self.setting("unpark.default_side")),
            mm_per_s_at_full=float(self.setting("startup.mm_per_s_at_full")),
            timeout_s=float(self.setting("unpark.timeout_s")))
        print(f"Starting in the bay - {self._unparking.summary()}")

    def _set_direction_from_the_bay(self, pose):
        """
        Settles which way round the lap to go from the wall the bay is on,
        before a wheel turns.

        The bay is stuck to the OUTER wall and the robot parks parallel to it,
        so the side that wall is on IS the lap direction - see
        travel_direction_beside_wall. Worth doing instead of leaving it to
        RacingLine.direction_for, which picks whichever direction needs the
        smaller turn: from inside a bay the robot is a long way off the line
        and up to a full bay-width of lateral error from it, and the two
        candidate headings it is choosing between are 180 degrees apart. Get
        that wrong and the round drives a confident lap the wrong way.
        """
        field = self.context.nav.map
        # nearest_outer_wall, not section_of alone: a bay more than 500mm
        # along its wall sits in a corner CELL, where section_of answers None,
        # and falling back to the racing line's guess there is exactly the
        # case this method exists to remove.
        section = section_of(pose.x, pose.y, field)
        wall = section or nearest_outer_wall(pose.x, pose.y, field)
        direction, alignment = travel_direction_beside_wall(wall, pose.heading)
        if direction is None:
            off_parallel = math.degrees(math.acos(min(1.0, alignment)))
            print(f"WARNING: the robot is {off_parallel:.0f}deg off parallel to the "
                  f"{wall} wall - too close to nose-on for the wall to say which way "
                  f"the lap runs, so the lap direction is the racing line's guess "
                  f"from the start pose")
            return
        self._bay_direction = direction
        was = self.direction
        self.direction = direction
        # Everything this concluded, in the terms you can check by eye against
        # the robot on the mat: which wall, which side of the robot it is on,
        # and what that makes the lap.
        side = "right" if direction > 0 else "left"
        cell = "" if section else "  (corner cell - wall found by distance)"
        agree = "" if was == direction else (
            f", overriding the racing line's guess of "
            f"{RacingLine.direction_name(was)}")
        print(f"Bay is on the {wall} wall{cell}")
        off_parallel = math.degrees(math.acos(min(1.0, alignment)))
        print(f"  heading {pose.heading:.0f}deg ({off_parallel:.0f}deg off parallel) "
              f"puts that wall on the robot's {side} "
              f"-> running {RacingLine.direction_name(direction)}{agree}")

    def _start_speed(self):
        """Held still while the exit is pending: it drives its own first tick."""
        if self._unparking is not None and not self._unparking.finished:
            return 0
        return super()._start_speed()

    def unparking_command(self, dt):
        """
        The exit's (steer, speed) for this tick - see
        PathDrivingTask._drive_unparking.
        """
        if self._unparking is None or self._unparking.finished:
            return None
        command = self._unparking.update(
            self.context.nav.get_pose(), dt,
            max_steer=self.pursuit.max_steer_command)
        if self._unparking.finished:
            self._rejoin_the_line()
        return command

    def _rejoin_the_line(self):
        """
        Picks the lap up from wherever the exit left the robot.

        setup() projected onto the racing line from inside the bay, which is
        several hundred millimetres off it and possibly pointing across it, so
        the direction and the lap counter it derived there describe nothing.
        Re-derived here, once, on the tick the exit ends - and the distance
        driven getting out does not count as lap.
        """
        pose = self.context.nav.get_pose()
        self.direction, source = self._lap_direction(pose)
        self.progress, self.lateral = self.path.project(pose.x, pose.y, self.direction)
        self.aim_progress = self.progress
        # ZERO AT THE BAY, NOT HERE. The exit hands back several hundred
        # millimetres PAST the bay, and zeroing the counter at that point puts
        # the end of the last lap there too - so the lap finishes with the bay
        # already behind the robot, which is the round going round again. The
        # getting-out does not count as lap distance, but the ground between
        # the bay and here does, because the next lap has to cover it.
        self.distance_driven = 0.0
        if self._start_point is not None:
            bay_progress, _ = self.path.project(*self._start_point, self.direction)
            self.distance_driven = self.path.gap(bay_progress, self.progress)
        self._rejoin_progress = self.progress
        print(f"Out of the bay at {pose} -> running "
              f"{RacingLine.direction_name(self.direction)} ({source})")
        self._warn_if_parking_cannot_reach_the_bay()

    def _warn_if_parking_cannot_reach_the_bay(self):
        """
        Says so when the last lap will end PAST the bay.

        The lap counter is zeroed here, where the exit ended - which is some
        way beyond the bay, because leaving it meant driving out of it. So a
        lap counted from here finishes past the bay too, and parking, which is
        not allowed to start until the laps are done, arms with the bay
        already behind the robot. It then has to go round again.

        parking.start_early_mm is what buys that back, and this says how much
        of it is needed: the distance from the bay to here, plus the run-up
        the approach itself wants.
        """
        if (self._bay_progress is None or self._rejoin_progress is None
                or self._warned_about_reach):
            return
        # Measured from where the EXIT handed back, not from wherever the
        # robot is now: that point is where the lap counter reads zero, so it
        # is also where the last lap will end.
        past = self.path.gap(self._bay_progress, self._rejoin_progress)
        if past <= 0.0:
            return
        self._warned_about_reach = True
        approach = float(self.setting("parking.follow_mm"))
        needed = past + approach
        early = float(self.setting("parking.start_early_mm"))
        if early >= needed:
            return
        print(f"WARNING: the exit left the robot {past:.0f}mm past the bay, so the "
              f"last lap ends there too. parking.start_early_mm is {early:.0f}mm but "
              f"needs about {needed:.0f} ({past:.0f} back to the bay + {approach:.0f} "
              f"of approach), or the round drives an extra lap before it can park.")

    def _lap_direction(self, pose):
        """
        Which way round the lap to go, from the most local evidence there is.

        THE LIDAR WINS. The bay is a slot in the OUTER wall, so the side that
        wall is on settles the direction - and the lidar measures that side in
        the robot's own frame, out of two sectors, with no pose and no map in
        the chain. The map can answer the same question, but only through the
        localizer, and the localizer is at its very worst here: a robot in a
        bay sees a scan unlike anywhere else on the field, and a pose that is
        900mm out reports a section, a wall and a direction with complete
        confidence. Measured on the robot: the pose put it against the centre
        block on the north wall while the lidar had the wall 116mm off its
        right, and the map-derived direction was therefore backwards.

        The map stays as the cross-check, because when the two disagree the
        pose is worth distrusting for the rest of the round too.

        I/O:
            return: (direction, one-line explanation of where it came from)
        """
        wall_side = self._unparking.wall_side if self._unparking else None
        if wall_side is not None and self._bay_direction is not None \
                and wall_side != self._bay_direction:
            print(f"WARNING: the lidar puts the outer wall "
                  f"{UnparkController.side_name(wall_side)} of the robot, the pose "
                  f"puts it {UnparkController.side_name(self._bay_direction)} - going "
                  f"with the lidar. The pose is suspect for the whole round: the "
                  f"parking bay and every pillar are placed through it.")
        if wall_side is not None:
            # Wall on the right is counter-clockwise, the same identity the
            # park drives on - see travel_direction_beside_wall.
            return wall_side, (f"outer wall {UnparkController.side_name(wall_side)} "
                               f"of the robot, by lidar")
        if self._bay_direction is not None:
            return self._bay_direction, "the wall the pose says the bay is on"
        return self.path.direction_for(pose), "the racing line's guess"
    # ========================================================================
    # CONTROL LOOP
    # ========================================================================

    def step(self):
        # Detection normally runs on VisionManager's thread and this does
        # nothing - the plan reads nav.blocks, which persists between frames,
        # not the current frame. The inline path is the fallback for when the
        # thread could not be started at all.
        if self.context.vision is None:
            if self.tick % CAMERA_EVERY_N_TICKS == 0:
                self._update_detections()
        super().step()

    def _update_detections(self):
        context = self.context
        if context.object_solver is None or context.camera is None:
            return
        try:
            # capture_for_blocks(), not capture_image() + transform_image():
            # this round reads the HSV frame and nothing else, and the full
            # pipeline costs ~19ms a frame to produce ~1.6ms of answer.
            hsv = context.camera.capture_for_blocks(
                with_display=context.object_solver.debug)
            if hsv is None:
                return
            context.nav.observe_blocks(context.object_solver.detect(
                hsv, display_image=context.camera.display_image))
        except Exception as e:
            print(f"WARNING: detection failed: {e!r}")

    def _track_progress(self, pose):
        """Where we are, as the base round tracks it, plus the pillars, the
        bay, and a new plan if this tick needs one."""
        super()._track_progress(pose)
        self._update_pillar_memory()
        self._look_for_bay(pose)
        reason = self._needs_replan(pose)
        if reason:
            self._replan(pose, reason)

    # ========================================================================
    # PILLARS
    # ========================================================================

    def _update_pillar_memory(self):
        """
        The pillars the plan is steering around, refreshed from the block map
        and HELD across the gaps in their own detection.

        Two things forget a pillar: driving past it, which is also what lets
        it be re-found on the next lap, and blocks.memory_s of silence, which
        bounds how long a detection that turned out to be wrong can keep
        bending the plan. Between those, a pillar that blinks out for a few
        frames stays exactly where it was and keeps its goals.

        Association is by distance in the FIELD frame rather than along the
        lap: two pillars can share a progress while sitting on opposite sides
        of the corridor, and merging those two would place one goal between
        them and drive at both.
        """
        now = time.monotonic()
        hold_s = float(self.setting("blocks.memory_s"))
        kept = [pillar for pillar in self._pillars
                if now - pillar.last_seen <= hold_s
                and self._gap_to(pillar) > -FORGET_BEHIND_MM]

        for block in self.context.nav.blocks.confirmed():
            if block.color not in SIDE_FOR_COLOR:
                continue
            index = self._remembered_index(block, kept)
            if index is not None:
                kept[index] = Pillar(block.x, block.y, block.color, now)
                continue
            # A pillar already behind the forget line is one just driven past.
            # BlockMap still has it and will keep offering it every tick, so
            # without this it is forgotten and immediately re-adopted, over and
            # over - which prints a fresh "pillar ahead" for something that is
            # a metre behind, and puts a goal in the plan for a pass that has
            # already happened.
            if self.path.gap(self.progress, self.path.project(
                    block.x, block.y, self.direction)[0]) <= -FORGET_BEHIND_MM:
                continue
            self._announce_pillar(block)
            kept.append(Pillar(block.x, block.y, block.color, now))
        self._pillars = kept

    def _progress_of(self, pillar):
        """
        Where a pillar sits along the lap.

        A Pillar keeps a field position rather than a progress, because a goal
        is a pose and not a point on a profile, so this projects on demand.
        The park's reach test wants the same number - see
        _pillar_before_the_bay.
        """
        at, _ = self.path.project(pillar.x, pillar.y, self.direction)
        return at

    def _gap_to(self, pillar):
        """How far ahead a pillar is, along the lap. Negative means passed."""
        return self.path.gap(self.progress, self._progress_of(pillar))

    @staticmethod
    def _remembered_index(block, pillars):
        """Which remembered pillar this detection refreshes, or None."""
        best, best_distance = None, SAME_PILLAR_MM
        for index, pillar in enumerate(pillars):
            distance = math.hypot(block.x - pillar.x, block.y - pillar.y)
            if distance < best_distance:
                best, best_distance = index, distance
        return best

    def _announce_pillar(self, block):
        gap = self.path.gap(self.progress,
                            self.path.project(block.x, block.y, self.direction)[0])
        side = "LEFT" if SIDE_FOR_COLOR[block.color] < 0 else "RIGHT"
        print(f"{block.color.name} pillar {gap:+.0f}mm ahead - passing on its "
              f"{side}")

    def _obstacles(self):
        """The remembered pillars in the form GoalPlanner wants."""
        return [Obstacle(pillar.x, pillar.y, SIDE_FOR_COLOR[pillar.color])
                for pillar in self._pillars]

    # ========================================================================
    # PLANNING
    # ========================================================================

    def _needs_replan(self, pose):
        """
        Why this tick wants a new plan, or "" to keep the one it has.

        Planning is cheap (under a millisecond for a normal lap) but not free,
        and a plan rebuilt every tick from a slightly different pose is a plan
        whose target point jitters. So it is rebuilt on a fixed cadence, and
        immediately whenever something it was built from has changed.
        """
        if self._plan is None:
            return "no plan"
        if len(self._pillars) != self._planned_pillars:
            return "pillars changed"
        if self._plan.remaining_mm(pose.x, pose.y) < self._min_plan_ahead_mm():
            return "ran off the end"
        if self._plan.cross_track_mm(pose.x, pose.y) > float(
                self.setting("goals.replan_cross_track_mm")):
            return "drifted off the plan"
        if (time.monotonic() - self._plan.planned_at
                > float(self.setting("goals.replan_interval_s"))):
            return "cadence"
        return ""

    def _min_plan_ahead_mm(self):
        """
        How much plan has to be left in front of the robot.

        Twice the lookahead: at one lookahead the target point is already
        sitting on the last sample of the plan, which means pure pursuit has
        stopped steering at the path and started steering at its end.
        """
        return 2.0 * self.pursuit.lookahead_distance(max(self.speed, 1))

    def _replan(self, pose, reason):
        self._plan = self.planner.plan(pose, self.direction, self._obstacles())
        self._planned_pillars = len(self._pillars)
        self._plans_made += 1
        self._replan_reason = reason
        if self._plan.compromised and self._plans_made % 20 == 1:
            print(f"WARNING: plan compromised - {self._plan.reason}")

    # ========================================================================
    # TARGET - the one thing this round changes about the driving
    # ========================================================================

    def target_point(self, pose):
        """
        The point to chase: one lookahead along the PLAN.

        The lookahead itself is unchanged - probed at speed, re-taken with the
        curvature of the racing line over that stretch, and capped by the
        parking approach exactly as the base round does it. Only the thing it
        is measured along is different, and that is the whole of the
        difference between this round and qualification.
        """
        reach = self.pursuit.lookahead_distance(self.speed)
        curvature = self.path.max_curvature_between(self.progress, reach,
                                                    self.direction)
        lookahead = self.pursuit.lookahead_distance(self.speed, curvature)
        _, reach_cap = self.parking_caps()
        if reach_cap is not None:
            lookahead = min(lookahead, float(reach_cap))

        # Kept current for the base round's own overlay and status line, which
        # still describe progress along the racing line.
        self.aim_progress = self.progress + lookahead

        if self._plan is None or len(self._plan) < 2:
            return self.path.point_at(self.aim_progress, self.direction)
        return self._plan.target_point(pose, lookahead)

    def _choose_speed(self, pose):
        """
        The base round's speed, cut back where the plan is in trouble.

        A compromised plan is one the planner could not give the clearance it
        asked for - a pillar the corridor cannot accommodate, or a pass the
        turning circle cannot hold. It is still the best plan available and it
        is still driven, but it is driven slowly: everything that makes a
        tight pass survivable - the lidar backstop having time to fire, a
        tracking error staying small in millimetres, the pose staying fresh -
        is bought with speed.
        """
        speed = super()._choose_speed(pose)
        if self._plan is not None and self._plan.compromised:
            speed = min(speed, int(self.setting("speed.compromised")))
        return speed

    # ========================================================================
    # PARKING
    # ========================================================================
    def _look_for_bay(self, pose):
        """
        Watch for the bay on EVERY lap, not just the last one.

        The bay does not move and the robot goes past it three times, so
        accumulating sightings turns the parking lap into "drive to a known
        place" instead of "find it now, with the clock running". Stops as soon
        as it is found.
        """
        if self._bay_finder is None or self._bay is not None:
            return
        lidar = self.context.lidar
        if lidar is None or section_of(pose.x, pose.y, self.context.nav.map) != self._start_section:
            return
        # Which way along the wall the robot is going, for a bay placed from a
        # single blade: the blade it meets first is the near one, so the bay
        # lies ahead of it. Same test BayFrame uses for its own `forward`.
        self._bay_finder.travel_sign = (
            1.0 if abs(angle_difference(pose.heading,
                                        wall_heading_of(self._start_section))) <= 90.0
            else -1.0)
        bay = self._bay_finder.observe(pose, lidar.get_scan())
        if bay is None:
            return

        self._bay = bay
        x, y = self._bay_finder.bay_point()
        self._bay_progress, _ = self.path.project(x, y, self.direction)
        # Now that the walls are known, put them on the map the filter matches
        # against. Until this point every beam that hit one was unexplained,
        # which is exactly the wrong thing to be happening as the robot lines
        # up to reverse between them. Raycast-only - see FieldMap.add_obstacle.
        self.context.nav.map.set_obstacles(
            wall_rects(bay[0], bay[1], self.context.nav.map,
                       bay_mm=self._bay_finder.bay_mm))
        print(f"Found the bay: {self._bay_finder.status_line()} "
              f"(lap {self.laps_done:.2f})")
        self._warn_if_parking_cannot_reach_the_bay()
        if self._bay_finder.from_single_blade:
            print(f"  Only one blade was ever seen, so the far wall is where "
                  f"the rules say it is, not where it was measured. The park "
                  f"lines up against the blade it DID see "
                  f"(parking.stage_at_wall), so an error in the assumed width "
                  f"moves the far end of the bay, not the end being aimed at.")

    def _start_parking(self):
        """
        Builds the parking sequence, once the laps are done.

        No bay position, no frame, no map: the sequence finds the bay itself
        from the side lidar. All this has to supply is which side the wall is
        on, and that comes from the lap direction - which is itself measured
        by lidar at the start of the round, so the pose is out of the chain
        end to end.
        """
        self._parking = ParkingSequence(
            lidar=self.context.lidar,
            compass=self.context.compass,
            wall_side=1.0 if self.direction > 0 else -1.0,
            wall_distance_mm=float(self.setting("parking.wall_distance_mm")),
            wall_gain=float(self.setting("parking.wall_gain")),
            wall_max_steer=float(self.setting("parking.wall_max_steer")),
            side_bearing_deg=float(self.setting("parking.side_bearing_deg")),
            side_sector_deg=float(self.setting("parking.side_sector_deg")),
            angle_gain=float(self.setting("parking.angle_gain")),
            angle_arc_deg=float(self.setting("parking.angle_arc_deg")),
            angle_min_points=int(self.setting("parking.angle_min_points")),
            angle_max_deg=float(self.setting("parking.angle_max_deg")),
            front_stop_mm=float(self.setting("parking.front_stop_mm")),
            front_sector_deg=float(self.setting("parking.front_sector_deg")),
            front_hold_s=float(self.setting("parking.front_hold_s")),
            body_stop_mm=float(self.setting("parking.body_stop_mm")),
            body_sector_deg=float(self.setting("parking.body_sector_deg")),
            inner_sector_deg=float(self.setting("parking.inner_sector_deg")),
            inner_slack_mm=float(self.setting("parking.inner_slack_mm")),
            trigger_below_mm=self.setting("parking.trigger_below_mm"),
            mouth_sector_deg=float(self.setting("parking.mouth_sector_deg")),
            blade_below_mm=self.setting("parking.blade_below_mm"),
            lidar_ahead_mm=float(self.setting("parking.lidar_ahead_mm")),
            turn_after_mm=self.setting("parking.turn_after_mm"),
            measure_bay=bool(self.setting("parking.measure_bay")),
            mouth_clear_mm=float(self.setting("parking.mouth_clear_mm")),
            bay_min_mm=float(self.setting("parking.bay_min_mm")),
            settle_max_mm=float(self.setting("parking.settle_max_mm")),
            settle_tolerance_mm=float(self.setting("parking.settle_tolerance_mm")),
            settle_angle_deg=float(self.setting("parking.settle_angle_deg")),
            settle_relax=float(self.setting("parking.settle_relax")),
            creep_max_mm=float(self.setting("parking.creep_max_mm")),
            turn_in_deg=float(self.setting("parking.turn_in_deg")),
            turn_in_steer=self.setting("parking.turn_in_steer"),
            turn_in_min_mm=self.setting("parking.turn_in_min_mm"),
            heading_gain=float(self.setting("parking.heading_gain")),
            nose_stop_mm=float(self.setting("parking.nose_stop_mm")),
            wheelbase_mm=float(self.setting("pursuit.wheelbase_mm")),
            max_road_wheel_deg=float(self.setting("pursuit.max_road_wheel_deg")),
            vision=self.context.vision,
            camera_confirms=bool(self.setting("parking.camera_confirms")),
            camera_bearing_deg=float(self.setting("parking.camera_bearing_deg")),
            speed=int(self.setting("parking.speed")),
            reverse_speed=int(self.setting("parking.reverse_speed")),
            servo_settle_s=float(self.setting("parking.servo_settle_s")),
            mm_per_s_at_full=float(self.setting("startup.mm_per_s_at_full")),
            # The road's nominal width, so station-keeping beside the bay has
            # a number to work with even if the follow never measured one.
            nominal_corridor_mm=(self.context.nav.map.outer
                                 - self.context.nav.map.inner),
            bay_ahead_mm=self._bay_ahead_mm(),
            timeout_s=float(self.setting("parking.timeout_s")))
        # The camera does nothing about the bay for the whole lap - a third
        # colour mask per frame that nothing reads. Switch it on now.
        if self.context.vision is not None:
            self.context.vision.watch_for_parking(True)
        side = "right" if self.direction > 0 else "left"
        print(f"Laps done - parking. Outer wall on the {side}.")
        print(f"  {self._parking.summary()}")

    def _bay_ahead_mm(self):
        """
        How far the bay is from here, by the lap counter alone.

        THE ROBOT KNOWS THE SPOT WITHOUT SEEING IT. The round started inside
        the bay and _rejoin_the_line zeroed distance_driven there, so the bay
        sits at every whole multiple of the lap length - and the approach
        arms parking.start_early_mm short of one of them. That distance is
        the answer, and it needs no lidar, no camera and no map.

        It is a FALLBACK, not the plan: the lidar's own trigger fires first
        whenever it recognises the bay, because a measured position beats a
        dead-reckoned one. This is what stops the round driving past a bay it
        simply failed to recognise - see ParkingSequence._follow.

        I/O:
            return: millimetres to the bay, or None when the counter cannot
                    say (no bay start, so nothing was ever zeroed there)
        """
        if self._start_point is None or self.path is None:
            return None
        remaining = self.laps_goal * self.path.length - self.distance_driven
        # Wrap into the lap: an attempt that starts after the counter has
        # passed the goal is measuring to the NEXT time the bay comes round.
        remaining %= self.path.length
        return remaining

    def parking_caps(self):
        """Step 0: slow down and shorten the lookahead as the bay comes up."""
        if self._parking is None or self._parking.finished:
            return (None, None)
        return self._parking.path_caps()

    def parking_command(self, dt):
        """
        The manoeuvre's steering and speed - and once the laps are done, it
        does not give them back.

        PARK, NO MATTER WHAT. There is no lapping to look for the bay, no
        going round for another run and no giving up and driving out the
        clock. The laps ending is the end of the driving part of the round;
        everything after it is parking, and the only ways out are DONE or the
        runner's own time limit.

        A failed attempt is retried ON THE SPOT, and the retry costs a short
        reverse rather than a lap. That reverse is not optional: nearly every
        way the approach fails leaves the robot out of road - hard against a
        corner, or too close to the wall to square up - and restarting from
        exactly there just fails again on the same tick. Backing up is what
        turns a retry into a different attempt instead of the same one.
        """
        if not self.setting("parking.enabled"):
            return None
        if self.distance_driven < self._park_after_mm():
            return None                      # laps not done - keep driving

        # ---- backing up between attempts ---------------------------------
        if self._park_backing_mm > 0.0:
            speed = int(self.setting("parking.reverse_speed"))
            self._park_backing_mm -= self.speed_mm_per_s(speed) * dt
            if self._park_backing_mm > 0.0:
                return (0.0, -speed)
            self._park_backing_mm = 0.0
            self._parking = None             # rebuilt below, from here

        if self._parking is None:
            self._start_parking()

        if self._parking.phase == ParkingSequence.ABORTED:
            return self._retry_the_park()

        if self._parking.finished:
            return (0.0, 0)                  # DONE - hold still, is_finished ends it

        pose = self.context.nav.get_pose()
        # ONCE IT IS DRIVING, IT KEEPS THE WHEELS. Everything from the pull
        # onward is an open sequence measured from one pose; handing the
        # wheels back to pure pursuit halfway through does not pause it, it
        # abandons it - and leaves a controller that is neither finished nor
        # driving, so the round cannot end either.
        return self._parking.update(pose, dt, max_steer=self.pursuit.max_steer_command)

    def _retry_the_park(self):
        """
        Back up and go again, however many times it takes.

        No cap. A cap is a decision to stop trying to park, and the round has
        already been told that parking is the only way it ends - the time
        limit is the bound, not an attempt count. What IS bounded is how far
        each retry reverses, so a robot wedged against something cannot back
        itself down the track.
        """
        reason = self._parking.reason or "aborted"
        self._park_attempts += 1
        self._park_backing_mm = float(self.setting("parking.retry_back_mm"))
        print(f"Parking attempt {self._park_attempts} failed ({reason}) - backing up "
              f"{self._park_backing_mm:.0f}mm and going again.")
        return (0.0, -int(self.setting("parking.reverse_speed")))

    def _park_after_mm(self):
        """
        How far the robot has to have driven before the manoeuvre may start.

        The whole lap distance, less `parking.start_early_mm` - which is how
        this parks SHORT of the point it set off from. The saving is real: the
        robot starts inside the bay and rejoins the line some way past it, so
        a lap counted from there ends past the bay too, and the approach then
        has to come round again to reach it.

        The early start is withheld while a pillar is still between here and
        the bay, so it means "once the last block is behind us" rather than
        "cut the last pillar short". A pillar the line is still dodging is a
        pillar the manoeuvre would have to steer around from inside its own
        approach, which it has no way to do - every step of it is an open arc.
        """
        full = self.laps_goal * self.path.length
        early = float(self.setting("parking.start_early_mm"))
        if early <= 0.0 or self._pillar_before_the_bay():
            return full
        return full - early

    def _pillar_before_the_bay(self):
        """
        Is a mapped pillar still between the robot and the bay?

        Answered in lap distance rather than in field position, so a pillar
        beside the bay but a lap away does not count. gap() wraps into +/-
        half a lap, so a bay that reads as behind the robot means the bay is
        not what is coming up next and there is nothing to wait for.

        A "pillar" within PILLAR_FREE_MM of the bay is NOT one, and has to be
        ignored here or it costs a whole lap. The field never puts a pillar
        within that distance of a parking space, so anything the map has there
        is a bay wall read as a block, or a misdetection. Waiting for it to be
        passed means waiting until the robot is past the BAY - at which point
        the early start it was gating has nothing left to be early about, and
        the round goes round again. That is not a hypothetical: it is the
        extra lap, and the reason the approach never gets to slide in.
        """
        if self._bay_progress is None:
            return False
        to_bay = self.path.gap(self.aim_progress, self._bay_progress)
        if to_bay <= 0.0:
            return False
        return any(0.0 < self.path.gap(self.aim_progress, at) < to_bay
                   and abs(self.path.gap(at, self._bay_progress)) > PILLAR_FREE_MM
                   for at in (self._progress_of(pillar)
                              for pillar in self._pillars))

    # ========================================================================
    # FINISHING
    # ========================================================================

    def is_finished(self):
        """
        The round ends when the robot is PARKED. That is the only ending.

        No extra laps and no giving up: once the laps are done the manoeuvre
        owns the wheels and keeps retrying until it closes - see
        parking_command. The runner's own time limit is the backstop, and it
        scores the laps already driven rather than throwing them away.
        """
        if self._stop_reason:
            print(f"Stopping: {self._stop_reason}")
            return True
        if not self.setting("parking.enabled"):
            return self.laps_done >= self.laps_goal
        return (self._parking is not None
                and self._parking.phase == ParkingSequence.DONE)

    # ========================================================================
    # REPORTING
    # ========================================================================

    def _plan_status(self):
        """
        What the plan is doing, in the one line that gets printed while the
        robot is driving.

        The clearance shown is the MEASURED one - the closest the swept
        footprint comes to a pillar over the whole plan - not the one the
        config asked for. That distinction is the entire point of the rewrite,
        so it is what the status line reports.
        """
        if self._plan is None:
            return "plan=none"
        gaps = []
        pillar_gap = getattr(self._plan, "pillar_gap_mm", float("inf"))
        if math.isfinite(pillar_gap):
            gaps.append(f"pillar {pillar_gap:.0f}mm")
        wall_gap = getattr(self._plan, "wall_gap_mm", float("inf"))
        if math.isfinite(wall_gap):
            gaps.append(f"wall {wall_gap:.0f}mm")
        passes = sum(1 for goal in self._plan.goals if goal.obstacle is not None)
        line = (f"plan={self._plan.length_mm:.0f}mm {passes // 2 or passes} "
                f"pass  {'  '.join(gaps)}")
        if self._plan.compromised:
            line = f"{line}  COMPROMISED({self._plan.reason})"
        return line

    def status(self):
        # A manoeuvre owns the wheels, so the lap's own trace says nothing
        # useful about what the robot is doing - report the manoeuvre's.
        if self._unparking is not None and self._unparking.active:
            return f"{super().status()}  {self._unparking.status_line()}"
        if self._parking is not None and self._parking.active:
            # The park's own trace: which phase, what the side lidar reads and
            # how square the body is to the wall. Tuning the five distances by
            # watching the robot is guesswork; this is the line that says
            # whether it triggered on the bay wall or on the outer one.
            return f"{super().status()}  {self._parking.status_line()}"
        line = (f"{super().status()}  {self.context.nav.blocks.summary()}"
                f"  {self._park_gate_status()}  {self._plan_status()}")
        if self.context.vision is not None:
            line = f"{line}  {self.context.vision.status_line()}"
        return line

    def _park_gate_status(self):
        """
        Why parking has not started yet, on every status line.

        A round that never parks and never says why is the hardest thing here
        to work on: the gate has four separate ways of staying shut and all of
        them used to be silent, so "it just keeps going" could be any of them.
        This puts the live answer in the trace instead.
        """
        if not self.setting("parking.enabled"):
            return "park=off"
        if self._park_backing_mm > 0.0:
            return f"park=backing {self._park_backing_mm:.0f}mm (try {self._park_attempts + 1})"
        if self._parking is not None:
            tries = "" if self._park_attempts == 0 else f" try {self._park_attempts + 1}"
            return f"park={self._parking.phase}{tries}"
        togo = self._park_after_mm() - self.distance_driven
        if togo > 0.0:
            held = " (pillar)" if self._pillar_before_the_bay() else ""
            return f"park=in {togo:.0f}mm{held}"
        return "park=starting"

    def _draw_overlay(self, canvas, to_px):
        """
        The racing line, the plan on top of it, and the goals the plan is
        made of.

        The goals are what is worth seeing here. The old view drew two bent
        lines and left you to infer from the gap between them that a dodge
        had been clamped; this one draws the pose the robot is actually
        aiming to be in beside each pillar, which is the thing that either
        clears it or does not.
        """
        super()._draw_overlay(canvas, to_px)
        if self._plan is not None:
            color = (60, 90, 235) if self._plan.compromised else (90, 200, 220)
            self._plan.draw(canvas, to_px, color=color)
        for pillar in self._pillars:
            spot = to_px(pillar.x, pillar.y)
            bgr = ((70, 200, 70) if pillar.color is Color.GREEN
                   else (60, 60, 210))
            cv2.circle(canvas, spot, 7, bgr, 2)

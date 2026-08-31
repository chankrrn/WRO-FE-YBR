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

One thing this round has to do before any of that is work out where it is.
The robot starts INSIDE a parking space, between two walls that stick 200mm
out from the outer wall and are not in the map the particle filter matches
against. Every beam that hits one is unexplained, so the filter either refuses
to converge or converges somewhere wrong - and a wrong pose at tick zero is a
wrong plan for the whole round. See _wait_for_localization.
"""
import math
import time
from collections import namedtuple

import cv2

from classes.block_map import BLOCK_SIZE_MM
from classes.goal_planner import GoalPlanner, Obstacle
from classes.parking import (BayFinder, BayFrame, ParkingController,
                             section_of, wall_rects)
from tasks.path_task import PathDrivingTask
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
        self._left_the_bay = False

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
            exit_mm=float(self.setting("goals.exit_mm")))
        print(self.planner)

        pose = self.context.nav.get_pose()
        self._start_section = section_of(pose.x, pose.y, self.context.nav.map)
        print(f"Parking bay expected in the {self._start_section} section")
        if self.setting("parking.enabled") and self._start_section is not None:
            self._bay_finder = BayFinder(
                self.context.nav.map, self._start_section,
                min_depth_mm=float(self.setting("parking.detect_min_depth_mm")),
                min_gap_mm=float(self.setting("parking.detect_min_gap_mm")),
                max_gap_mm=float(self.setting("parking.detect_max_gap_mm")),
                min_scans=int(self.setting("parking.detect_min_scans")))
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
    # STARTING FROM INSIDE THE PARKING SPACE
    # ========================================================================

    def _wait_for_localization(self):
        """
        Find the robot - after driving it out of the parking space, if that is
        where it started.

        The round begins with the robot parked between the two bay walls.
        Those walls are 100mm tall, stick 200mm out from the outer wall, and
        are NOT in FieldMap: the filter only knows the outer box and the
        centre block. So from inside the bay a large share of the lidar's
        beams are returns the map cannot explain, and an unexplained beam does
        not merely add noise - it actively pushes weight onto whatever pose
        would explain it, which is a pose somewhere else on the mat.

        The result is a filter that either sits below min_pose_confidence
        until the timeout and starts anyway on a pose it does not believe, or
        converges hard onto a wrong one. Either way `direction_for()` may pick
        the wrong way round the loop and the first plan is built from a lie.

        Waiting longer cannot fix it, because nothing improves while the robot
        is stationary between two walls the map does not contain. Driving out
        does: three or four hundred millimetres forward and the scan is the
        ordinary corridor the map describes perfectly.

        So: creep out first, THEN localize. The creep is open-loop - it is the
        one stretch of the round where there is no pose worth steering on -
        and deliberately short and slow, since it is driven blind.
        """
        if not self.setting("start.in_parking_bay"):
            return super()._wait_for_localization()

        self._creep_out_of_the_bay()
        # Start the search over from scratch. Whatever the filter believed
        # from inside the bay was formed against a map missing the two walls
        # in front of it, so it is not a prior worth keeping - it is the thing
        # being corrected.
        self.context.nav.start(zones=self.context.nav.map.start_zones())
        pose = super()._wait_for_localization()
        self._left_the_bay = True
        print(f"Out of the parking space and localized: {pose}")
        return pose

    def _creep_out_of_the_bay(self):
        """
        Drives straight forward far enough to be clear of the bay walls.

        Open-loop and blind on purpose: there is no pose to steer on yet, and
        a steering correction computed from a pose that is wrong is worse than
        no correction at all. Straight ahead out of a parking space is the one
        manoeuvre that needs neither.

        Distance is dead-reckoned from the commanded speed, which is rough -
        but the only thing it has to be right about is "further than the bay
        walls are deep", and they are 200mm deep against a default of 600mm.
        Overshooting into the corridor is harmless; stopping short is not, so
        this errs long.

        The filter is fed the motion as it happens, so the pose it starts from
        afterwards is at least in the right postcode even before the first
        scan lands.
        """
        distance = float(self.setting("start.bay_exit_mm"))
        speed = int(self.setting("start.bay_exit_speed"))
        rate = self.speed_mm_per_s(abs(speed))
        if rate <= 0.0:
            print("WARNING: start.bay_exit_speed is 0 - not leaving the bay")
            return

        print(f"Starting inside the parking space - creeping {distance:.0f}mm "
              f"forward at speed {speed} before trusting the pose")
        self.context.motor.steer_center()
        self.context.motor.drive(0.0, speed)
        driven, last = 0.0, time.monotonic()
        # A time bound as well as a distance one: if the wheels are not
        # actually turning, dead reckoning never reaches the distance and this
        # would drive into the wall forever.
        deadline = last + distance / rate * 2.5 + 1.0
        while driven < distance and time.monotonic() < deadline:
            time.sleep(0.02)
            now = time.monotonic()
            step = rate * (now - last)
            last = now
            driven += step
            self.context.nav.report_motion(step, 0.0)
        self.context.motor.drive(0.0, 0)
        print(f"  crept {driven:.0f}mm")

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

    def _gap_to(self, pillar):
        """How far ahead a pillar is, along the lap. Negative means passed."""
        at, _ = self.path.project(pillar.x, pillar.y, self.direction)
        return self.path.gap(self.progress, at)

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

    def _start_parking(self):
        """Builds the manoeuvre once the laps are done and the bay is known."""
        _, _, heading = self.path.pose_at(self._bay_progress, self.direction)
        frame = BayFrame(self._bay[0], self._bay[1], self.context.nav.map,
                         heading, 1.0 if self.direction > 0 else -1.0)
        offset = self.setting("parking.stage_along_offset_mm")
        straight = self.setting("parking.straight_mm")
        self._parking = ParkingController(
            frame, bay_mm=self._bay_finder.bay_mm,
            turn_radius_mm=self.pursuit.min_turn_radius_mm,
            line_depth_mm=self.path.map.outer - self.path.half,
            turn_deg=float(self.setting("parking.turn_deg")),
            stage_depth_mm=float(self.setting("parking.stage_depth_mm")),
            stage_along_offset_mm=None if offset is None else float(offset),
            park_depth_mm=float(self.setting("parking.park_depth_mm")),
            end_offset_mm=float(self.setting("parking.end_offset_mm")),
            straight_mm=None if straight is None else float(straight),
            approach_mm=float(self.setting("parking.approach_mm")),
            approach_lookahead_mm=float(self.setting("parking.approach_lookahead_mm")),
            speed=int(self.setting("parking.speed")),
            approach_speed=int(self.setting("parking.approach_speed")))
        print(f"Laps done - parking. Racing line is "
              f"{self.path.map.outer - self.path.half:.0f}mm off the wall, "
              f"turning radius {self.pursuit.min_turn_radius_mm:.0f}mm")
        print(f"  {self._parking.summary()}")

    def parking_caps(self):
        """Step 0: slow down and shorten the lookahead as the bay comes up."""
        if self._parking is None or self._parking.finished:
            return (None, None)
        return self._parking.path_caps()

    def parking_command(self, dt):
        """
        The manoeuvre's steering and speed for this tick, or None to let the
        racing line keep driving - see PathDrivingTask._drive_parking.
        """
        if not self.setting("parking.enabled") or self._bay is None:
            return None
        if self.laps_done < self.laps_goal:
            return None
        if self._parking is None:
            self._start_parking()
        if self._parking.finished:
            return None
        pose = self.context.nav.get_pose()
        # The bay frame's `s` is just a field coordinate, so it means nothing
        # while the robot is somewhere else on the mat - and would happily
        # read as "past the staging point" from the far side of the field.
        if section_of(pose.x, pose.y, self.context.nav.map) != self._bay[0]:
            return None
        return self._parking.update(pose, dt, max_steer=self.pursuit.max_steer_command)

    # ========================================================================
    # FINISHING
    # ========================================================================

    def is_finished(self):
        """
        The round is over when the robot is PARKED, not when the laps are.

        With parking on, running out of laps is not a reason to stop - it is
        the cue to start looking for somewhere to stop. If the bay is never
        found the robot keeps lapping and the runner's own time limit ends the
        round, which scores the laps rather than throwing them away.
        """
        if self._stop_reason:
            print(f"Stopping: {self._stop_reason}")
            return True
        if not self.setting("parking.enabled"):
            return self.laps_done >= self.laps_goal
        if self._parking is not None:
            return self._parking.finished
        # Laps done, no bay found yet: keep going round and keep looking. Not
        # forever, though - without this bound a round with no bay in front of
        # it (a bench test, a practice mat without the walls) never ends at
        # all and just burns the clock.
        return self.laps_done >= self.laps_goal + float(self.setting("parking.extra_laps"))

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
        line = (f"{super().status()}  {self.context.nav.blocks.summary()}"
                f"  {self._plan_status()}")
        if self.context.vision is not None:
            line = f"{line}  {self.context.vision.status_line()}"
        return line

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

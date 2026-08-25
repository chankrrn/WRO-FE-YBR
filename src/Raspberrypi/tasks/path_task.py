"""
The driving half of both rounds: follow a loop with pure pursuit, count laps,
and do not hit anything.

What the two rounds share is everything except one method. Qualification
drives the racing line as generated; the final round shifts it sideways to
pass each pillar on the required side. That is the ONLY difference in the
driving, so it is the only thing `target_lateral_mm()` exists for - see
tasks/final/task.py.

The loop per tick:

    pose      ask NavigationManager where we are
    progress  project the pose onto the path -> how far round, how far off
    laps      accumulate progress, wrapped, to count laps
    target    a point one lookahead ahead on the path, shifted sideways by
              whatever target_lateral_mm() asks for
    steer     pure pursuit onto that point
    speed     back off for corners, for a bad pose, and for anything the lidar
              sees close ahead

Placement is unconstrained: the robot may be set down anywhere on the mat
facing any direction. Setup waits for the localizer to converge, then picks
whichever way round the loop needs the smaller turn, so the round starts
clockwise or counter-clockwise depending on how the robot was placed.
"""
import math
import time
from dataclasses import replace

from classes.debug_view import DebugView
from classes.pure_pursuit import PurePursuit
from classes.steering_calibrator import SteeringCalibrator
from classes.racing_line import RacingLine
from tasks.base_task import Task
from utils.angle_utils import clamp

# ============================================================================
# Defaults. Anything here can be overridden per round in its config.toml -
# these are the values used when a key is missing from the file.
# ============================================================================
DEFAULTS = {
    "laps.goal": 3,

    "speed.base": 70,              # straights
    "speed.corner": 60,            # sharpest part of a corner
    "speed.lost": 50,              # pose not trustworthy
    "speed.minimum": 40,           # never crawl slower than this while driving

    "path.wall_margin_mm": None,   # None = centre the loop in the corridor
    "path.corner_radius_mm": None,  # None = biggest that clears the block
    "path.resolution_mm": 20.0,

    "pursuit.wheelbase_mm": 165.0,
    "pursuit.lookahead_base_mm": 260.0,
    "pursuit.lookahead_per_speed_mm": 3.0,
    "pursuit.lookahead_min_mm": 250.0,
    "pursuit.lookahead_max_mm": 700.0,
    "pursuit.max_road_wheel_deg": 30.0,
    "pursuit.max_steer_command": 80.0,
    "pursuit.rear_axle_offset_mm": 0.0,

    # How fast the steering command is allowed to move, in command units per
    # second. None scales it to the rack: 3 x max_steer_command, i.e. lock to
    # centre in a third of a second.
    #
    # A RATE LIMIT, not a filter. A low-pass would lag every steering change
    # including the slow ones that make up normal cornering, costing tracking
    # everywhere to fix a problem that only happens on sharp steps. This does
    # nothing at all until the command tries to move faster than the limit,
    # which is exactly when the servo snaps and shakes the camera.
    "steer.max_rate_units_s": None,
    # Cap on the lookahead inside a bend, as a fraction of the bend's radius.
    # Lower it if the robot still cuts corners; raise it if it weaves.
    "pursuit.corner_lookahead_fraction": 0.8,
    # Seconds of actuator lag to steer ahead of. The wheels do not reach the
    # angle you asked for instantly, so by the time they get there the robot
    # has moved - steering off the pose it has THEN, rather than the pose it
    # has now, cancels that delay. Set it to the steering system's response
    # time; test_steering.py measures it.
    "pursuit.lag_compensation_s": 0.0,

    "safety.min_pose_confidence": 0.35,
    "safety.lost_timeout_s": 4.0,
    "safety.front_slow_mm": 450.0,
    "safety.front_stop_mm": 220.0,
    "safety.front_sector_deg": 25.0,
    # Stopping in front of something is only half a backstop - a stopped robot
    # cannot drive away from what stopped it. These back it out instead.
    "safety.stuck_timeout_s": 1.0,
    "safety.reverse_time_s": 1.2,
    "safety.reverse_speed": 50,

    "startup.localize_timeout_s": 8.0,
    # The rules only allow the robot to be set down in the four non-corner
    # cells. Seeding the filter's search there instead of over the whole ring
    # locks on faster and cannot pick a pose that was never legal.
    "startup.assume_start_zone": True,
    "startup.mm_per_s_at_full": 700.0,   # drive speed 100 -> this many mm/s

    # Final round only - see tasks/final/task.py. They live here so that
    # setting() has a default for them whatever config file is loaded.
    "blocks.clearance_mm": 180.0,        # gap between robot and pillar face
    # The straight part of the dodge: this far either side of the pillar the
    # line runs parallel to the racing line at the full offset. approach_mm
    # and past_mm are the transitions leading into and out of it.
    "blocks.padding_mm": 150.0,
    "blocks.approach_mm": 900.0,         # length of the easing-in transition
    # How far off a pillar may be mapped, overriding block_map's own
    # MAX_MAPPING_RANGE_MM. None leaves that alone. This is the real ceiling
    # on approach_mm - a sweep cannot start before the pillar exists.
    "blocks.map_range_mm": None,
    "blocks.past_mm": 250.0,             # length of the easing-out transition
    # How long a pillar keeps steering the line after its track is lost. A
    # dodge must not be cancelled by a dropped frame - that is a collision -
    # so it is held, and this bounds how long a wrong detection can hold it.
    "blocks.memory_s": 1.5,
    "blocks.wall_clearance_mm": 220.0,   # never dodge closer than this to a wall
    # Pure pursuit steers the robot's CENTER onto the target point, not its
    # edge - without this, clearance_mm is the gap from the robot's centerline
    # to the block, not from its actual body, and the real gap comes up short
    # by however wide the chassis is. Half the robot's own width, widest point
    # (mirrors, wheels) to centerline - measure it, do not guess.
    "blocks.robot_half_width_mm": 100.0,
    "blocks.red_extra_clearance_mm": 0.0,     # extra padding on RED dodges only
    "blocks.green_extra_clearance_mm": 0.0,   # extra padding on GREEN dodges only
    # How much of the robot's own minimum turning circle to keep in hand where
    # a dodge pushes the line INWARD through a corner, which tightens the arc
    # it has to drive one mm for one. 1.0 is the bare geometric limit - the
    # line comes out at exactly the tightest arc the steering can hold, with
    # nothing left for tracking error. This ONLY ever limits a pillar that has
    # to be passed on the inside of a bend; a dodge on a straight, or one to
    # the outside of a bend, is never touched by it. See
    # FinalTask._corner_room_mm.
    "blocks.turn_radius_margin": 1.15,
    # Where the two inner control points of a dodge's easing-in and easing-out
    # curves sit, as a fraction of the transition's length. Both ends of each
    # curve are horizontal whatever these are, so they do not decide whether
    # the line joins smoothly - only how much of the transition is spent
    # turning. 1/3 is the plain S-curve; lower turns earlier and more evenly,
    # higher holds the straight longer and turns harder in the middle.
    "blocks.bezier_lead": 1.0 / 3.0,
    "blocks.bezier_settle": 1.0 / 3.0,
    # The steepest the line is aimed to lean, in degrees off the racing line,
    # when crossing from one pillar's dodge to the next. The curve between two
    # pillars is given the length it needs to hold this BEFORE either pillar's
    # flat part gets any of the gap - past about 45 degrees pure pursuit stops
    # following the line and starts cutting across it. Where two pillars are
    # too close for it at any length, the curve takes what there is and leans
    # harder; steep beats absent. See FinalTask._share_gap.
    "blocks.max_lean_deg": 45.0,
    # Over how much driving the line closes the gap when the profile moves
    # under it - a pillar appearing, or one being dropped. The dodge itself is
    # a function of where the pillars are and nothing else; this is the only
    # place the robot's own position enters, and it exists so that a pillar
    # confirmed late slides the line across instead of stepping it. Too short
    # and a pop-in is still a yank; too long and the line is lazy about
    # reaching a dodge it has just learned about. See FinalTask._settle_bias.
    "blocks.catchup_mm": 800.0,

    # Final round only - the parking bay. See classes/parking.py for the
    # geometry these describe and why the manoeuvre is shaped the way it is.
    "parking.enabled": True,
    # Whether the line is kept off the outer wall across the WHOLE starting
    # section before BayFinder has located the bay. Off, because the
    # competition never puts a pillar within PILLAR_FREE_MM of a parking
    # space: wherever the bay turns out to be, no pillar's pass is beside it,
    # so the line is near the racing line there and already clear of a bay
    # wall. Guarding speculatively costs a pillar dodge in that section about
    # half its clearance, on every lap, to protect against a case the field
    # layout rules out. Turn it on for a mat where that does not hold. The
    # window around the bay once it IS found is always guarded either way.
    "parking.guard_before_found": False,
    # Real gap from the robot's BODY to the tips of the bay walls while
    # driving PAST them. They stick 200mm out from the outer wall, so the
    # ordinary wall_clearance_mm - which assumes a flat wall - would happily
    # steer a dodge straight into one.
    "parking.wall_margin_mm": 40.0,
    # How far the BODY reaches sideways from the point that tracks the path.
    # Not half the width: the robot is only that narrow when it is perfectly
    # square to the line. Yawed by t it reaches half_width*cos(t) plus its
    # nose overhang*sin(t), which for a 240mm body at 15 degrees is about
    # 110mm, and the corner-to-centre half-diagonal is 134mm. Add what the
    # path follower's own tracking error contributes and this is the number
    # that decides whether a dodge clears a bay wall - sizing the clamp off
    # robot_half_width_mm instead lets a yawed robot clip one while the
    # commanded line looks perfectly legal.
    "parking.body_reach_mm": 130.0,
    # How much lap either side of the bay the tighter clamp applies over, once
    # the bay has been found. Before that it covers the whole start section,
    # because the first pass at the bay happens before anything knows where it
    # is. Narrower means less argument with a pillar that wants a wall-side
    # pass in the same section.
    "parking.window_mm": 500.0,
    # A bay is only believed after this many scans agree about it.
    "parking.detect_min_scans": 3,
    # Reversing speed for the manoeuvre, and the slower creep used for the
    # last stretch up to the staging point. The approach is deliberately slow:
    # at racing speed one control tick is 8mm, and every millimetre of
    # overshoot there lands on the tightest clearance of the whole manoeuvre.
    # How much further the robot will keep lapping, looking for the bay,
    # once the laps are done. The fallback is deliberately "keep driving" -
    # laps already scored are worth more than a blind park - but it cannot be
    # unbounded, or a round with no bay in front of it never ends at all.
    "parking.extra_laps": 1.5,

    # ---- the four-step park itself ----------------------------------------
    # 2  full lock toward the wall, reversing, until the yaw reaches this
    "parking.turn_deg": 45.0,
    # 1  how far off the outer wall the robot lines up before it starts
    "parking.stage_depth_mm": 350.0,
    # 1  how far PAST the far wall's inner face the rear axle lines up.
    # Unset solves it from the rest, so the manoeuvre lands centred; set a
    # number to place it by hand. Zero is "rear axle level with the far
    # wall", which is where a driver would start - but at the measured
    # turning radius the solved value is about +180mm, because the straight
    # in step 3 carries the robot a long way back before it stops.
    "parking.stage_along_offset_mm": None,
    # 3  ends when the closing arc would finish at this depth. Unset lets the
    # length follow from the depths; a number overrides it.
    "parking.straight_mm": None,
    "parking.park_depth_mm": 105.0,
    # where the parked body sits along the bay, + toward the far wall
    "parking.end_offset_mm": 0.0,
    # 0  how much run-up gets the slow, short-lookahead treatment, and what
    # that lookahead is. Long enough to slide in from the racing line without
    # leaning more than approach_lean_deg.
    "parking.approach_mm": 900.0,
    "parking.approach_lookahead_mm": 200.0,
    "parking.speed": 25,
    "parking.approach_speed": 25,
    # How far out from the outer wall a return has to be to count as a bay
    # wall rather than the wall itself, and the gap between the two clusters
    # that reads as a bay.
    "parking.detect_min_depth_mm": 120.0,
    "parking.detect_min_gap_mm": 250.0,
    "parking.detect_max_gap_mm": 400.0,
}


class PathDrivingTask(Task):
    """
    Base for any round that drives laps of the racing line.

    Subclasses override `target_lateral_mm()` to bend the line, and otherwise
    inherit the whole control loop. Requires the lidar - without a pose there
    is no path following, so this round cannot run blind.
    """

    name = "path"
    requires_lidar = True

    def __init__(self, context, config=None, **kwargs):
        super().__init__(context, **kwargs)
        self.config = config
        self.laps_goal = int(self.setting("laps.goal"))

        self.path = None
        self.pursuit = None
        self.direction = 1

        self.progress = 0.0          # travel-frame progress this tick
        self.aim_progress = 0.0      # progress of the point being chased
        self.lateral = 0.0           # how far off the line we are, + right
        self.distance_driven = 0.0   # along the path, for lap counting
        self.speed = 0
        self.steer_command = 0.0
        self.target = None

        self._last_tick_at = None
        self._front_mm = None        # forward lidar clearance, refreshed per tick
        self._lost_since = None
        self._stop_reason = None
        self._blocked_since = None
        self._reversing_until = None
        self.calibrator = None
        self._steer_rate_limit = None    # units/s, resolved at setup
        self._debug_view = None

    # ========================================================================
    # CONFIG
    # ========================================================================

    def setting(self, key):
        """A tunable, from this round's config.toml or the shared default."""
        default = DEFAULTS.get(key)
        return default if self.config is None else self.config.get(key, default)

    # ========================================================================
    # SETUP
    # ========================================================================

    def setup(self):
        context = self.context
        context.motor.steer_center()

        self.path = RacingLine(
            field_map=context.nav.map,
            wall_margin_mm=self.setting("path.wall_margin_mm"),
            corner_radius_mm=self.setting("path.corner_radius_mm"),
            resolution_mm=self.setting("path.resolution_mm"))
        self.pursuit = PurePursuit(
            wheelbase_mm=self.setting("pursuit.wheelbase_mm"),
            lookahead_base_mm=self.setting("pursuit.lookahead_base_mm"),
            lookahead_per_speed_mm=self.setting("pursuit.lookahead_per_speed_mm"),
            lookahead_min_mm=self.setting("pursuit.lookahead_min_mm"),
            lookahead_max_mm=self.setting("pursuit.lookahead_max_mm"),
            max_road_wheel_deg=self.setting("pursuit.max_road_wheel_deg"),
            max_steer_command=self.setting("pursuit.max_steer_command"),
            rear_axle_offset_mm=self.setting("pursuit.rear_axle_offset_mm"),
            corner_lookahead_fraction=self.setting("pursuit.corner_lookahead_fraction"))

        # Always on: it costs one arithmetic step per tick and turns every
        # lap into a measurement of the one gain that cannot be derived.
        self.calibrator = SteeringCalibrator(
            wheelbase_mm=self.pursuit.wheelbase_mm,
            max_steer_command=self.pursuit.max_steer_command,
            assumed_max_road_wheel_deg=self.pursuit.max_road_wheel_deg)

        rate = self.setting("steer.max_rate_units_s")
        self._steer_rate_limit = (float(rate) if rate
                                  else 3.0 * self.pursuit.max_steer_command)
        self.steer_command = 0.0

        print(self.path)
        print(f"Steering slew limit: {self._steer_rate_limit:.0f} units/s "
              f"({self.pursuit.max_steer_command / self._steer_rate_limit * 1000:.0f}ms "
              f"centre to full lock)")
        self._warn_if_corners_too_tight()
        self._warn_if_lookahead_unusable()

        pose = self._wait_for_localization()
        # Only now: everything the camera maps before the filter converges is
        # rejected on the way in, so there is nothing to gain by looking
        # earlier and a core to save by not.
        if context.vision is not None:
            context.vision.resume()
        self.direction = self.path.direction_for(pose)
        self._warn_if_started_outside_a_zone(pose)
        self.progress, self.lateral = self.path.project(pose.x, pose.y, self.direction)
        self.aim_progress = self.progress
        self.distance_driven = 0.0
        self._last_tick_at = time.monotonic()

        print(f"Placed at {pose} -> running {RacingLine.direction_name(self.direction)}, "
              f"{self.laps_goal} laps of {self.path.length / 1000:.2f}m")
        self.speed = int(self.setting("speed.base"))
        context.motor.forward(self.speed)

    def _wait_for_localization(self):
        """
        Holds the robot still until the particle filter has actually found it.
        Driving off on a pose that has not converged is how you end up
        confidently steering into the center block.

        A timeout is not fatal: the filter reports its own confidence, and the
        loop already slows down when that is low, so it is better to roll
        cautiously than to refuse to start a competition run.
        """
        timeout = float(self.setting("startup.localize_timeout_s"))
        minimum = float(self.setting("safety.min_pose_confidence"))
        deadline = time.monotonic() + timeout

        if self.setting("startup.assume_start_zone"):
            zones = self.context.nav.map.start_zones()
            self.context.nav.start(zones=zones)
            print(f"Searching the {len(zones)} legal start cells...")

        pose = self.context.nav.get_pose()
        while time.monotonic() < deadline:
            pose = self.context.nav.update()
            if pose.confidence >= minimum:
                print(f"Localized in {timeout - (deadline - time.monotonic()):.1f}s")
                return pose
            time.sleep(0.05)

        print(f"WARNING: still unsure of the pose after {timeout:.0f}s "
              f"(confidence {pose.confidence:.2f}) - starting slowly anyway")
        return pose

    def _warn_if_lookahead_unusable(self):
        """
        Says out loud what the lookahead settings actually do.

        Two ways to tune a number that has no effect. The corner cap overrides
        the speed-based lookahead inside every bend, and on this loop the bends
        are most of the lap - so a big lookahead_base_mm can be quietly ignored
        almost everywhere while looking like the thing being tuned. And a
        lookahead approaching the lap length aims the robot most of a lap
        ahead, which on a closed loop is a point behind the centre block.
        """
        pursuit, path = self.pursuit, self.path
        speed = float(self.setting("speed.base"))
        asked = pursuit.lookahead_distance(speed)
        corner_cap = pursuit.corner_lookahead_fraction * path.corner_radius

        if asked > path.length * 0.25:
            print(f"WARNING: lookahead {asked:.0f}mm is a quarter of the "
                  f"{path.length:.0f}mm lap - the target point is most of a lap ahead, "
                  f"which on a closed loop is across the field. Lower "
                  f"pursuit.lookahead_base_mm.")
        if pursuit.lookahead_min_mm > corner_cap:
            print(f"WARNING: in corners the lookahead is capped at {corner_cap:.0f}mm "
                  f"({pursuit.corner_lookahead_fraction} x {path.corner_radius:.0f}mm "
                  f"radius), below lookahead_min_mm={pursuit.lookahead_min_mm:.0f}. "
                  f"Most of this lap is corner, so lookahead_base_mm/min_mm barely "
                  f"matter - tune pursuit.corner_lookahead_fraction instead.")
        print(f"Lookahead: {asked:.0f}mm on the straights, "
              f"{corner_cap:.0f}mm in the corners")

    def _warn_if_started_outside_a_zone(self, pose):
        """
        The robot can only be placed in one of the four non-corner cells, so a
        pose in a corner means the filter has not really converged. It is not
        worth refusing to run over - the round drives the same loop wherever
        it starts - but it is worth saying, because it usually means the lidar
        or the mat dimensions are wrong.
        """
        for name, low, high in self.context.nav.map.start_zones():
            if low[0] <= pose.x <= high[0] and low[1] <= pose.y <= high[1]:
                print(f"Start cell: {name}")
                return
        print(f"WARNING: localized to {pose}, which is not one of the four legal "
              f"start cells - check the field dimensions in classes/field_map.py")

    def _warn_if_corners_too_tight(self):
        """
        A corner tighter than the robot's own turning circle cannot be driven,
        however good the controller is. Worth saying out loud at setup rather
        than discovering it as understeer into the block.
        """
        needed = self.pursuit.min_turn_radius_mm
        if self.path.corner_radius < needed:
            print(f"WARNING: path corner radius {self.path.corner_radius:.0f}mm is "
                  f"tighter than the robot's {needed:.0f}mm turning circle - "
                  f"raise path.corner_radius_mm or check pursuit.max_road_wheel_deg")

    # ========================================================================
    # CONTROL LOOP
    # ========================================================================

    def step(self):
        context = self.context
        now = time.monotonic()
        dt = 0.0 if self._last_tick_at is None else now - self._last_tick_at
        self._last_tick_at = now
        # Once per tick, before anything asks. Both the stop test and the
        # speed ramp want it, and two reads inside one tick can disagree
        # about what is in front of the robot.
        self._front_mm = self._measure_front_clearance()

        # Report BOTH how far we drove and how far we turned. The filter uses
        # them to predict with, and get_pose() uses them to carry the last
        # scan-derived pose forward to now - which is what lets this loop run
        # at 50Hz off a lidar that only produces a pose at 10Hz.
        distance = self._travelled(dt)
        context.nav.report_motion(distance, self._turned(distance))
        pose = context.nav.get_pose()

        self._track_progress(pose)

        # Parking owns the wheels outright once it starts: it is driving arcs
        # and short reverse strokes that pure pursuit has no way to express,
        # and the safety stops below would fight it (the front sector is
        # SUPPOSED to be full of bay wall). Odometry above is untouched, so
        # the filter keeps tracking through the manoeuvre - see
        # _drive_parking.
        if self._drive_parking(dt):
            if context.debug:
                self.show_debug()
            return

        self._update_lost_state(pose, now)

        self.target = self.target_point(pose)
        # Steer off where the robot WILL be once the wheels have caught up.
        wanted = self.pursuit.steering(self._lead_pose(pose), self.target)
        # ... then hold the servo to a sane speed. `steer_command` is the
        # LIMITED value from here on, so the odometry, the calibrator and the
        # status line all describe what the wheels were actually told to do.
        self.steer_command = self._limit_steer_rate(wanted, dt)

        if self._reverse_out(now):
            # Backing away from something; _reverse_out has already commanded
            # the motor, so leave the normal speed logic alone this tick.
            if context.debug:
                self.show_debug()
            return

        # Fit the steering gain off the RAW filter pose - the extrapolated one
        # is dead-reckoned from the gain we are trying to measure.
        self.calibrator.observe(context.nav.get_pose(max_age_s=None, extrapolate=False),
                                self.steer_command)

        self.speed = self._choose_speed(pose)
        cap, _ = self.parking_caps()
        if cap is not None:
            self.speed = min(self.speed, int(cap))
        # One serial message for both, and none at all when neither moved -
        # see MotorManager.drive().
        context.motor.drive(self.steer_command, self.speed)

        if context.debug:
            self.show_debug()

    def parking_caps(self):
        """
        What the parking approach wants the path follower limited to, as
        (speed, lookahead_mm) - either may be None for "no limit".

        Separate from parking_command because this applies while the PATH is
        still driving. A long lookahead is what makes pure pursuit cut a
        corner, and cutting it on the way into the bay means arriving at the
        staging pose off line and off square, which nothing downstream can
        correct - every step of the manoeuvre is an open arc measured from
        that pose.

        Nothing here: the qualification round never parks.
        """
        return (None, None)

    def parking_command(self, dt):
        """
        The parking manoeuvre's (steer_command, speed) for this tick, or None
        when it is not driving.

        Nothing here: the qualification round never parks. The final round
        overrides it - see tasks/final/task.py.
        """
        return None

    def _drive_parking(self, dt):
        """
        Hands the wheels to the parking manoeuvre for a tick.

        The controller returns a command rather than touching the motor
        itself, so that `steer_command` and `speed` stay the single record of
        what the wheels were told - which is what _travelled() and _turned()
        dead-reckon from, and what the status line reports. The road-wheel
        angle has to be published to the pursuit object explicitly, because
        the usual writer of it (PurePursuit.steering) is not running.

        I/O:
            return: True if this tick was spent parking
        """
        command = self.parking_command(dt)
        if command is None:
            return False
        steer, speed = command
        self.steer_command = clamp(float(steer), -self.pursuit.max_steer_command,
                                   self.pursuit.max_steer_command)
        self.speed = int(speed)
        self.pursuit.set_road_wheel_command(self.steer_command)
        self.context.motor.drive(self.steer_command, self.speed)
        return True

    def _travelled(self, dt):
        """
        Distance covered since the last tick, from the commanded speed.

        Zero with the drive motor disabled (--no-drive): the commanded speed
        then says nothing about how fast a hand is pushing the robot, and
        feeding the filter motion the wheels never made walks the pose away
        from where the robot really is. The lidar carries it instead, at scan
        rate rather than tick rate.
        """
        if not self.context.motor.drive_enabled:
            return 0.0
        full_speed = float(self.setting("startup.mm_per_s_at_full"))
        return self.speed / 100.0 * full_speed * dt

    def _limit_steer_rate(self, wanted, dt):
        """
        Caps how far the steering command may move in one tick.

        A servo driven straight from a pure-pursuit output takes whatever step
        the controller asks for, and a big step makes it snap - which shakes
        the mast, which blurs the camera frame the pillar detection depends on.
        Bounding the change per second turns that snap into a sweep without
        touching the slow, ordinary steering that makes up a corner.

        Costs a little tracking, because a correction that wanted to happen now
        happens over the next few ticks instead. Measure before lowering it:
            python test_steering.py --sweep steer.max_rate_units_s 60,120,240
        """
        if not self._steer_rate_limit or dt <= 0.0:
            return wanted
        step = self._steer_rate_limit * dt
        return clamp(wanted, self.steer_command - step, self.steer_command + step)

    def _lead_pose(self, pose):
        """
        The pose projected forward by `pursuit.lag_compensation_s`.

        Steering is an actuator with a delay: ask for 20 degrees and the wheels
        arrive there a tenth of a second later, by which time the robot is
        somewhere else. Feeding the controller the present pose therefore
        produces a correction that is always one lag too late, and a late
        correction is an over-correction - the robot swings past the line,
        corrects back, and weaves.

        Projecting the pose forward by the lag closes that gap: the command
        that lands is the one that was right for the moment it lands. This is
        the same dead reckoning NavigationManager uses between scans, just run
        a little further into the future, so it costs nothing.

        Left at 0 this returns the pose unchanged.
        """
        lead_s = float(self.setting("pursuit.lag_compensation_s"))
        if lead_s <= 0.0 or not self.speed:
            return pose

        distance = self.speed / 100.0 * float(self.setting("startup.mm_per_s_at_full")) * lead_s
        turn = self._turned(distance)
        midpoint = math.radians(pose.heading + turn / 2.0)
        return replace(pose,
                       x=pose.x + distance * math.sin(midpoint),
                       y=pose.y + distance * math.cos(midpoint),
                       heading=(pose.heading + turn) % 360.0)

    def _turned(self, distance_mm):
        """
        Yaw change over that distance, from the bicycle model and the steering
        angle we last commanded. Free - the road-wheel angle is already known,
        it is what we asked the servo for.
        """
        if not distance_mm:
            return 0.0
        road_wheel = math.radians(self.pursuit.last_road_wheel_deg)
        return math.degrees(distance_mm / self.pursuit.wheelbase_mm * math.tan(road_wheel))

    def _track_progress(self, pose):
        """
        Folds this tick's projection into the lap counter.

        The delta is wrapped into +/- half a lap, so crossing the seam at the
        start of the loop counts as a small step forward rather than a whole
        lap backwards. Reversing counts backwards, which is what you want.
        """
        progress, self.lateral = self.path.project(pose.x, pose.y, self.direction)
        self.distance_driven += self.path.gap(self.progress, progress)
        self.progress = progress

    def _update_lost_state(self, pose, now):
        minimum = float(self.setting("safety.min_pose_confidence"))
        if pose.confidence >= minimum:
            self._lost_since = None
            return
        if self._lost_since is None:
            self._lost_since = now
        elif now - self._lost_since > float(self.setting("safety.lost_timeout_s")):
            self._stop_reason = (f"lost for {now - self._lost_since:.1f}s "
                                 f"(confidence {pose.confidence:.2f})")

    def _reverse_out(self, now):
        """
        Backs the robot away from whatever the front sector is stuck against.

        Without this the front-clearance stop is a trap: it sets the speed to
        zero, and a stopped robot cannot drive out of what stopped it, so the
        round sits there until it times out. Reversing for a moment and trying
        again is what a driver would do.

        The steering is INVERTED while reversing. Backwards, a given steering
        angle swings the nose the other way, so mirroring the pursuit command
        rotates the robot toward its target rather than further from it.

        I/O:
            return: True if this tick was spent reversing
        """
        if self._reversing_until is not None:
            if now < self._reversing_until:
                self.speed = -int(self.setting("safety.reverse_speed"))
                self.context.motor.drive(-self.steer_command, self.speed)
                return True
            self._reversing_until = None
            self._blocked_since = None

        front = self._front_clearance_mm()
        if front is None or front >= float(self.setting("safety.front_stop_mm")):
            self._blocked_since = None
            return False

        if self._blocked_since is None:
            self._blocked_since = now
            return False
        if now - self._blocked_since < float(self.setting("safety.stuck_timeout_s")):
            return False

        self._reversing_until = now + float(self.setting("safety.reverse_time_s"))
        print(f"Blocked at {front:.0f}mm - backing out")
        return True

    # ========================================================================
    # TARGET - the one thing the final round changes
    # ========================================================================

    def target_point(self, pose):
        """
        The point to chase: one lookahead along the path, shifted sideways.

        The lookahead is probed at speed first, then re-taken with the
        curvature over that stretch, so a corner shortens it - see
        PurePursuit.lookahead_distance.
        """
        reach = self.pursuit.lookahead_distance(self.speed)
        curvature = self.path.max_curvature_between(self.progress, reach, self.direction)
        lookahead = self.pursuit.lookahead_distance(self.speed, curvature)
        _, reach_cap = self.parking_caps()
        if reach_cap is not None:
            lookahead = min(lookahead, float(reach_cap))

        ahead = self.progress + lookahead
        # Published because the lateral target is a function of THIS point,
        # not of where the robot is - anything reasoning about when a shift in
        # the line starts has to measure from the same place, a lookahead
        # further on. See FinalTask._update_ramp_anchors.
        self.aim_progress = ahead
        return self.path.point_at(ahead, self.direction, self.target_lateral_mm(ahead))

    def target_lateral_mm(self, progress):
        """
        How far to the right of the racing line to aim at that progress.

        Zero here: the qualification round drives the line as generated. The
        final round overrides this to step around pillars, which is the whole
        of the difference between the two rounds' driving.
        """
        return 0.0

    # ========================================================================
    # SPEED
    # ========================================================================

    def _choose_speed(self, pose):
        """
        Slowest of everything that wants us slow: corner geometry, a shaky
        pose, and whatever the lidar can see straight ahead.
        """
        base = float(self.setting("speed.base"))
        corner = float(self.setting("speed.corner"))
        minimum = float(self.setting("speed.minimum"))

        speed = self._corner_speed(base, corner)
        if pose.confidence < float(self.setting("safety.min_pose_confidence")):
            speed = min(speed, float(self.setting("speed.lost")))

        front = self._front_clearance_mm()
        if front is not None:
            if front < float(self.setting("safety.front_stop_mm")):
                return 0
            slow = float(self.setting("safety.front_slow_mm"))
            if front < slow:
                # Ramp down over the last stretch rather than stepping, or the
                # robot lurches every time a wall comes into the front sector.
                stop = float(self.setting("safety.front_stop_mm"))
                fraction = (front - stop) / max(1.0, slow - stop)
                speed = min(speed, minimum + (speed - minimum) * fraction)

        if self._stop_reason:
            return 0
        return int(round(clamp(speed, minimum, 100.0)))

    def _corner_speed(self, base, corner):
        """
        Interpolates between the straight and corner speeds by how sharp the
        path gets within one lookahead - so the robot is already slow when it
        arrives at the corner rather than braking in it.
        """
        reach = self.pursuit.lookahead_distance(self.speed)
        curvature = self.path.max_curvature_between(self.progress, reach, self.direction)
        if curvature <= 0.0:
            return base
        sharpness = clamp(curvature * self.path.corner_radius, 0.0, 1.0)
        return base + (corner - base) * sharpness

    def _front_clearance_mm(self):
        """
        Closest lidar return in the forward sector this tick, or None if it is
        blind. Measured once per tick by step() - see _measure_front_clearance.
        """
        return self._front_mm

    def _measure_front_clearance(self):
        """
        Reads the forward sector out of the lidar.

        Called once a tick and cached, because _reverse_out and _choose_speed
        both want the answer and each call copies the whole 360-slot scan out
        from under the lidar thread's lock and re-applies the staleness mask.
        Correctness, not just cost: the two are deciding whether to stop and
        how fast to go, and they should be deciding it about the same frame.
        """
        lidar = self.context.lidar
        if lidar is None:
            return None
        half = float(self.setting("safety.front_sector_deg"))
        distance, _ = lidar.get_min_distance(-half, half)
        return None if math.isnan(distance) else distance

    # ========================================================================
    # FINISHING
    # ========================================================================

    @property
    def laps_done(self):
        return self.distance_driven / self.path.length if self.path else 0.0

    def is_finished(self):
        if self._stop_reason:
            print(f"Stopping: {self._stop_reason}")
            return True
        return self.laps_done >= self.laps_goal

    def finish(self):
        super().finish()
        if self._debug_view is not None:
            self._debug_view.close()
        if self.context.object_solver is not None:
            self.context.object_solver.close_debug()
        if self.calibrator is not None:
            print(self.calibrator.report())
        print(f"{self.laps_done:.2f} laps driven "
              f"({self.distance_driven / 1000:.1f}m of {self.laps_goal} x "
              f"{self.path.length / 1000:.2f}m)" if self.path else "no path built")

    # ========================================================================
    # REPORTING
    # ========================================================================

    def status(self):
        pose = self.context.nav.get_pose(max_age_s=None)
        return (f"[{self.elapsed:5.1f}s] lap {self.laps_done:4.2f}/{self.laps_goal}  "
                f"({pose.x / 10:+6.1f},{pose.y / 10:+6.1f})cm  "
                f"off-line {self.lateral / 10:+5.1f}cm  "
                f"{self.pursuit.status_line()}  speed={self.speed}"
                f"{'(off)' if not self.context.motor.drive_enabled else ''}  "
                f"conf={pose.confidence:.2f}")

    def show_debug(self):
        """
        Field view with the racing line and the pursuit target drawn on. ESC
        (window mode) ends the round, same as any other stop reason.

        Also the one place the object solver's two windows get painted: its
        detect() runs on the vision thread, which must not touch HighGUI, so
        it only renders (see ObjectSolver.show_debug). They go up before the
        field view, so DebugView's cv2.waitKey() paints all three at once.
        """
        if self._debug_view is None:
            self._debug_view = DebugView(self.context.nav,
                                         ascii_mode=self.context.ascii_debug)
        solver = self.context.object_solver
        if solver is not None and solver.debug:
            # ascii mode means there is no window loop to pump them - and no
            # waitKey either, so anything already up has to come down.
            if self._debug_view.ascii_mode:
                solver.close_debug()
            else:
                solver.show_debug()
        if not self._debug_view.show(draw=self._draw_overlay):
            self._stop_reason = self._stop_reason or "debug window closed (ESC)"

    def _draw_overlay(self, canvas, to_px):
        import cv2

        self.path.draw(canvas, to_px)
        if self.target is not None:
            cv2.circle(canvas, to_px(*self.target), 6, (60, 220, 240), 2)
            cv2.line(canvas, to_px(*self.target),
                     to_px(*self.path.point_at(self.progress, self.direction)),
                     (60, 220, 240), 1)

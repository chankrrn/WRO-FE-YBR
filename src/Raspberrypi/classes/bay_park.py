"""
Nose-in bay park, rewritten as five plain legs.

    FIND      drive the wall at road_middle_mm, parallel, until the near
              blade steps into the side beam (or, lacking that, until a
              known bay_ahead_mm runs out) - then place the bay's middle
              directly, since its WIDTH is a field constant and not
              something worth re-measuring off one noisy beam
    SETTLE    past the bay the beam is on clean wall again: get to the
              middle of the road and square to it, and stay there
    BACK      reverse straight, heading held, until the axle is one turn
              radius short of the bay's middle
    TURN_IN   forward at full lock toward the wall, 90 degrees on the compass
    DRIVE_IN  forward straight until the NOSE is nose_stop_mm off the outer
              wall - which, having turned in from the bay's middle, is the
              middle of the bay
    WIGGLE    optional, off unless wiggle_steps is non-empty: drives a
              FULLY CUSTOM sequence of legs after DRIVE_IN stops - each one
              its own steer angle and distance, in whatever order and
              however many are configured - one shared speed throughout
    DONE / ABORTED

ONE BLADE, NOT TWO. An earlier version of this hunted both blades with the
side beam and measured the gap between them directly - a scripted test (feed
the state machine the documented geometry with nothing else in the loop)
found that the SAME sector width that has to be wide enough to reliably see
a thin (~10mm) blade at all also eats into the apparent width of the gap
BETWEEN the two blades from both ends, and a confirm window sized for one
problem broke the other: real runs either never saw a blade, or saw both
blades merge into one. classes/parking.py already carries the fix for
exactly this, in BayFinder's _from_one_blade: the bay's width is not a
measurement, it is a rule fixed by the field, so ONE blade plus that
constant is enough, and is more robust than two. This file takes the same
position - see NOMINAL_BAY_MM.

WHY THE ORDER IS THIS ORDER. Everything from BACK onward is open loop: a
measured reverse, a turn counted on the compass, a straight run in. None of
it can correct a pose error, so the pose it starts from is the whole park.
SETTLE exists to make that pose a known one - middle of the road, square to
the wall - and it deliberately happens PAST the bay, where the side beam is
back on clean wall and means what it says. Alongside the bay it does not:
the blades stand 200mm proud, so the beam is looking at a blade, and a
follow that believes it steers away from the thing it is parking in.

THE GEOMETRY, with this robot's numbers (road 1000mm, blades 200mm proud,
bay 200mm deep, axle-to-nose 200mm, turn radius about 204mm at full lock):

    settle at 500mm from the wall, square
    reverse to one radius short of the bay's middle
    turn 90 degrees: the axle swings 204mm toward the wall and 204mm along
        it, so it finishes about 296mm out and level with the bay's middle
    drive in: the nose leads the axle by 200mm, so it enters the mouth
        (200mm out) almost at once and stops 20mm off the wall

The turn is what spends the road, and it spends a whole radius of it. That
is why SETTLE targets the MIDDLE of the road and not the old 450mm: from
500mm the turn finishes with 296mm still in hand, and the nose crosses the
mouth under power rather than arriving there already committed.

This is a drop-in for ParkingSequence - same update/active/finished/phase/
reason/summary/status_line/path_caps - so tasks/final can build either.
"""

import math

from utils.angle_utils import angle_difference, clamp

# ============================================================================
# Geometry that is MEASURED, not tuned. Defaults only; the task passes the
# round's own numbers in.
# ============================================================================
ROAD_MIDDLE_MM = 500.0          # half of the 1000mm road
BLADE_PROUD_MM = 200.0          # how far a bay wall stands off the outer wall
NOSE_STOP_MM = 20.0             # gap from the NOSE to the outer wall at rest

# WIGGLE - an optional pass AFTER the nose stops, to nudge the car straighter
# and more central. A LIST of legs, each its own steer angle and distance,
# driven in order with one shared speed - not a fixed shape, because a
# parallel-park-style correction is rarely symmetric (the first leg to free
# the car is often not the same length or angle as the one that squares it
# up). Empty by default - it only runs where a config actually lists steps,
# because it spends road behind the car and a bay that is already square
# does not need it. See BayPark.__init__'s wiggle_steps for the shape of one
# entry, and tasks/final/config.toml for how to write the list out in TOML.
WIGGLE_STEPS = ()
WIGGLE_SPEED = 55

# The field's own gap between the two blades' inner faces, and the blades'
# own thickness ALONG the wall - classes/parking.py's NOMINAL_BAY_MM and
# WALL_THICKNESS_MM, repeated here rather than imported so this file has no
# dependency on the old manoeuvre it replaces. Keep them in step if the
# field geometry is re-measured.
NOMINAL_BAY_MM = 340.0
WALL_THICKNESS_MM = 10.0

# A blade is in the beam when the side range drops this far below the wall
# distance being followed. The blades stand 200mm proud, so the step is a
# big one - well clear of the +/-10mm the C1 wobbles by, and of the 40mm the
# follow itself is allowed to drift.
BLADE_STEP_MM = 110.0

# The blade has to hold for this far before it counts. At the crossing the
# beam's footprint straddles a 10mm-thick blade, so without this a single
# blade reads as three.
#
# MUST BE SMALLER THAN THE APPARENT DWELL, NOT JUST SMALLER THAN THE BLADE.
# A scripted test - feed the state machine the documented geometry with
# nothing else in the loop - caught this directly with an earlier, narrow
# sector: a blade is physically ~10mm thick, and a sensor that only looks
# straight along its bearing is "on" it for about that same 10mm of travel,
# LESS than a 40mm confirm window, so the filter meant to reject flicker
# rejected every real blade instead and FIND never left. See
# SIDE_SECTOR_DEG for the other half of the fix - widening the sector so the
# blade is in view for comfortably longer than this.
BLADE_CONFIRM_MM = 40.0

# SETTLE's gate: how close to the middle of the road, and how square, before
# the open-loop half is allowed to start.
SETTLE_TOLERANCE_MM = 35.0
SETTLE_ANGLE_DEG = 3.0
# How much road it may spend trying. The bar widens across it (see _settle),
# so a nearly-square robot leaves at once and only a crooked one uses it all.
SETTLE_MAX_MM = 700.0
SETTLE_RELAX = 0.6

# How far past the bay's far edge (blade included) to be, by odometry, before
# testing squareness at all - see _settle. A little slack past the raw edge
# for the axle-lead and sector-width biases _place_bay does not fully remove.
PAST_BAY_MARGIN_MM = 60.0

# How far the wide side sector reports the near blade EARLY - see
# _place_bay. A mat run parking short of the bay's true middle is this bias
# uncorrected; raise it to push the whole manoeuvre further along the wall,
# lower it (or make negative) if a run ever overshoots the other way. Start
# at 0 and tune from what the mat actually does - there is no clean formula
# for it, since it depends on the side sector's real footprint and the
# blade's actual distance, not just the nominal geometry.
BLADE_LEAD_MM = 0.0

# The wall follow. Two terms: distance and yaw. A follow with only a distance
# cannot tell 500mm-and-parallel from 500mm-and-crabbing.
WALL_GAIN = 0.10                # steer command per mm of distance error
ANGLE_GAIN = 0.9                # steer command per degree of yaw
WALL_MAX_STEER = 28.0

# HOW FAST THE WALL FOLLOW MAY MOVE THE WHEELS, in command units per second.
#
# _drive_parking hands the manoeuvre the wheels outright and returns its
# command straight to the motor - it does not pass through the round's own
# steer.max_rate_units_s slew limit (tasks/path_task.py), which only wraps
# the pure-pursuit path. So a single noisy or momentarily-wrong side reading
# - a dropped scan, a beam that grazed a blade at an angle instead of square
# - would otherwise snap the servo to near full lock in one tick with
# nothing upstream to catch it. This is the same guard, applied locally.
STEER_SLEW_DEG_S = 150.0

# Holding a straight line on the compass, forward or reversing.
HEADING_GAIN = 1.2
HEADING_MAX_STEER = 20.0

# The side beam.
#
# WIDE, DELIBERATELY - the opposite of what a "measure exactly where the
# blade starts" instinct suggests. get_min_distance reports the CLOSEST
# return over the whole sector, so a wider sector sees a blade EARLIER, from
# further along the approach, which is a real cost: the edge is logged tens
# of millimetres before the axle is actually level with it. But the sector
# has to be wide enough that the blade's apparent dwell - how long the
# reading stays "on it" as the robot drives past - comfortably exceeds
# EDGE_CONFIRM_MM, or the confirm filter above rejects every real blade as
# noise. A blade is only ~10mm thick; at 8 degrees a single-ray sensor sees
# it for about that same 10mm and BLADE_CONFIRM_MM=40 throws it away
# outright - caught by a scripted test that fed the state machine the
# documented geometry directly and watched FIND never leave. 25 degrees
# against this geometry (bay at ~400-500mm) buys on the order of 150-200mm
# of dwell, the same trade the rest of this codebase already makes (see
# side_sector_deg elsewhere and the "beam leads the axle" comments this
# shares the consequence with).
#
# THE COST IS NOT FULLY COMPENSATED. _edges() corrects for the beam leading
# the axle (lidar_ahead_mm), but the wide sector ALSO sees the blade before
# the beam is square on it, which is a further early bias this does not
# measure or remove - it is folded into bay_centre_s as a constant-ish
# offset instead, the same way the old code's own APPROACH_BLADE_MM and
# "the arc reaches about 210mm ahead" comments describe living with it. If
# the robot backs in short of the bay's true middle on the mat, this is the
# first place to look, and BLADE_STEP_MM/SIDE_SECTOR_DEG are what to trade
# against each other to shrink it.
SIDE_BEARING_DEG = 90.0
SIDE_SECTOR_DEG = 25.0
# The arc the wall's yaw is fitted through - wide, because a line fit wants
# points, and it is only ever used where the wall is clean.
ANGLE_ARC_DEG = 25.0
ANGLE_MIN_POINTS = 10
ANGLE_MAX_DEG = 25.0            # more than this is a corner, not the wall

FRONT_SECTOR_DEG = 12.0
FRONT_STOP_MM = 400.0           # a corner ahead while still looking for the bay

MIN_VALID_MM = 60.0
MAX_VALID_MM = 6000.0
USABLE_FOV_DEG = 100.0          # past this the scan sees the robot's own mast

# How far the whole search may run before giving up on this lap.
FIND_MAX_MM = 4000.0
DEFAULT_TIMEOUT_S = 45.0

MIN_MOVE_SPEED = 55             # below this the wheels do not turn at all

# Simple scripted park sequence: straighten to the wall, drive forward,
# turn to the opposite side of the bay, drive forward to the middle island,
# reverse away from it, turn at angle, then straighten. These are the values
# to tune on the mat and keep in the final task config.
STRAIGHTEN_TO_WALL_MM = 180.0
FORWARD_DISTANCE_MM = 140.0
TURN_OPPOSITE_DEG = 30.0
FORWARD_TO_MIDDLE_MM = 260.0
REVERSE_AWAY_MM = 200.0
FINAL_TURN_DEG = 28.0
FINAL_TURN_DISTANCE_MM = 180.0
FINAL_STRAIGHTEN_MM = 70.0
MIDDLE_STRAIGHTEN_AFTER_MM = 80.0


def _valid(distance):
    return (distance is not None and not math.isnan(distance)
            and MIN_VALID_MM < distance < MAX_VALID_MM)


class BayPark:
    """Nose-in park: settle in the middle of the road, back up, turn in."""

    FIND, SETTLE, BACK, TURN_IN, DRIVE_IN, WIGGLE, DONE, ABORTED = (
        "find", "settle", "back", "turn_in", "drive_in", "wiggle",
        "done", "aborted")
    DRIVING = (FIND, SETTLE, BACK, TURN_IN, DRIVE_IN, WIGGLE)

    def __init__(self, lidar=None, compass=None, vision=None,
                 wall_side=1.0,
                 road_middle_mm=ROAD_MIDDLE_MM,
                 nose_stop_mm=NOSE_STOP_MM,
                 wiggle_steps=WIGGLE_STEPS,
                 wiggle_speed=WIGGLE_SPEED,
                 blade_step_mm=BLADE_STEP_MM,
                 bay_mm=NOMINAL_BAY_MM,
                 bay_ahead_mm=None,
                 blade_lead_mm=BLADE_LEAD_MM,
                 settle_tolerance_mm=SETTLE_TOLERANCE_MM,
                 settle_angle_deg=SETTLE_ANGLE_DEG,
                 settle_max_mm=SETTLE_MAX_MM,
                 settle_relax=SETTLE_RELAX,
                 wall_gain=WALL_GAIN,
                 angle_gain=ANGLE_GAIN,
                 wall_max_steer=WALL_MAX_STEER,
                 steer_slew_deg_s=STEER_SLEW_DEG_S,
                 heading_gain=HEADING_GAIN,
                 side_bearing_deg=SIDE_BEARING_DEG,
                 side_sector_deg=SIDE_SECTOR_DEG,
                 angle_arc_deg=ANGLE_ARC_DEG,
                 angle_min_points=ANGLE_MIN_POINTS,
                 angle_max_deg=ANGLE_MAX_DEG,
                 front_stop_mm=FRONT_STOP_MM,
                 find_max_mm=FIND_MAX_MM,
                 lidar_ahead_mm=120.0,
                 robot_front_mm=200.0,
                 wheelbase_mm=165.0,
                 max_road_wheel_deg=50.0,
                 turn_in_deg=90.0,
                 straighten_to_wall_mm=STRAIGHTEN_TO_WALL_MM,
                 forward_distance_mm=FORWARD_DISTANCE_MM,
                 turn_opposite_deg=TURN_OPPOSITE_DEG,
                 forward_to_middle_mm=FORWARD_TO_MIDDLE_MM,
                 reverse_away_mm=REVERSE_AWAY_MM,
                 middle_straighten_after_mm=MIDDLE_STRAIGHTEN_AFTER_MM,
                 final_turn_deg=FINAL_TURN_DEG,
                 final_turn_distance_mm=FINAL_TURN_DISTANCE_MM,
                 final_straighten_mm=FINAL_STRAIGHTEN_MM,
                 speed=55, reverse_speed=60,
                 mm_per_s_at_full=390.0,
                 min_move_speed=MIN_MOVE_SPEED,
                 timeout_s=DEFAULT_TIMEOUT_S):
        self.lidar = lidar
        self.compass = compass
        self.vision = vision
        # +1 when the outer wall - and so the bay - is on the robot's RIGHT.
        # Every signed decision in here is written for that and multiplied by
        # this, so there is exactly one place the mirror lives.
        self.wall_side = 1.0 if wall_side >= 0 else -1.0

        self.road_middle_mm = float(road_middle_mm)
        self.nose_stop_mm = float(nose_stop_mm)
        # Each entry: {"steer_deg": ..., "distance_mm": ..., "reverse": bool}.
        # "reverse" defaults to False (forward) when a step omits it.
        # Accepts plain dicts (as TOML's [[array of tables]] parses to) or
        # any object with the same three keys/attributes.
        self.wiggle_steps = [
            {
                "steer_deg": float(self._step_field(step, "steer_deg", 0.0)),
                "distance_mm": float(self._step_field(step, "distance_mm", 0.0)),
                "reverse": bool(self._step_field(step, "reverse", False)),
            }
            for step in wiggle_steps
        ]
        self.wiggle_speed = float(wiggle_speed)
        self.blade_step_mm = float(blade_step_mm)
        # The field's own gap between the two blades - a constant, not
        # something measured off one noisy beam. See the module docstring.
        self.bay_mm = float(bay_mm)
        # A dead-reckoned distance to the bay from wherever this laps
        # counter or an earlier BayFinder sighting says it is, or None. Pure
        # backstop: the side beam finding the near blade always wins if it
        # fires first - see _find.
        self.bay_ahead_mm = None if bay_ahead_mm is None else float(bay_ahead_mm)
        # How far the wide side sector reports the near blade EARLY - a
        # constant to add back into near_blade_s, not noise to filter. See
        # _place_bay. Tune this directly against how short/long a mat run
        # parks: short of the middle -> raise it, long -> lower it.
        self.blade_lead_mm = float(blade_lead_mm)
        self.settle_tolerance_mm = float(settle_tolerance_mm)
        self.settle_angle_deg = float(settle_angle_deg)
        self.settle_max_mm = float(settle_max_mm)
        self.settle_relax = max(0.0, float(settle_relax))
        self.wall_gain = float(wall_gain)
        self.angle_gain = float(angle_gain)
        self.wall_max_steer = float(wall_max_steer)
        self.steer_slew_deg_s = float(steer_slew_deg_s)
        self.heading_gain = float(heading_gain)
        self.side_bearing_deg = float(side_bearing_deg)
        self.side_sector_deg = float(side_sector_deg)
        self.angle_arc_deg = float(angle_arc_deg)
        self.angle_min_points = int(angle_min_points)
        self.angle_max_deg = float(angle_max_deg)
        self.front_stop_mm = float(front_stop_mm)
        self.find_max_mm = float(find_max_mm)
        self.lidar_ahead_mm = float(lidar_ahead_mm)
        self.robot_front_mm = float(robot_front_mm)
        self.wheelbase_mm = float(wheelbase_mm)
        self.max_road_wheel_deg = float(max_road_wheel_deg)
        self.turn_in_deg = float(turn_in_deg)
        self.straighten_to_wall_mm = float(straighten_to_wall_mm)
        self.forward_distance_mm = float(forward_distance_mm)
        self.turn_opposite_deg = float(turn_opposite_deg)
        self.forward_to_middle_mm = float(forward_to_middle_mm)
        self.reverse_away_mm = float(reverse_away_mm)
        self.middle_straighten_after_mm = float(middle_straighten_after_mm)
        self.final_turn_deg = float(final_turn_deg)
        self.final_turn_distance_mm = float(final_turn_distance_mm)
        self.final_straighten_mm = float(final_straighten_mm)
        self.speed = int(speed)
        self.reverse_speed = int(reverse_speed)
        self.mm_per_s_at_full = float(mm_per_s_at_full)
        self.min_move_speed = int(min_move_speed)
        self.timeout_s = float(timeout_s)

        self.phase = self.FIND
        self.reason = None
        self.max_steer_seen = 40.0

        # Odometry along the wall, in AXLE coordinates, running for the whole
        # manoeuvre. Every distance the park measures is a difference of two
        # readings of this, so a constant scale error cancels out of the
        # geometry even though it does not cancel out of the legs.
        self.s_mm = 0.0
        self.leg_mm = 0.0           # distance since the current phase began

        self.side_mm = float("nan")
        self.wall_angle_deg = float("nan")

        # The near blade's position in axle coordinates, and the bay's
        # middle derived from it plus the known bay_mm - see _find.
        self.near_blade_s = None
        self.bay_centre_s = None
        self._on_blade = False
        self._pending_edge = None   # (kind, s) awaiting its confirm window

        self.back_target_s = None
        self.turned_deg = 0.0
        self._turn_from = None
        self.hold_heading = None    # the wall's heading, banked while clean
        self._heading_samples = []
        self._settle_blind_mm = 0.0

        # WIGGLE's own bookkeeping - see _wiggle. Index into wiggle_steps.
        self._wiggle_index = 0

        self._elapsed = 0.0
        self._steer = 0.0
        self._last_dt = 0.02

    # ------------------------------------------------------------------
    # WHAT THE TASK ASKS
    # ------------------------------------------------------------------
    @property
    def active(self):
        return self.phase in self.DRIVING

    @property
    def finished(self):
        return self.phase in (self.DONE, self.ABORTED)

    def path_caps(self):
        """Nothing: this drives itself from the moment it starts."""
        return (None, None)

    def turn_radius_mm(self, steer=None):
        """
        Rear-axle radius at full lock - what the turn-in costs in road, and
        so how far short of the bay's middle the reverse has to stop.
        """
        command = self.max_steer_seen if steer is None else abs(steer)
        wheel = math.radians(command / max(self.max_steer_seen, 1.0)
                             * self.max_road_wheel_deg)
        if wheel < 1e-6:
            return 1e9
        return self.wheelbase_mm / math.tan(wheel)

    def summary(self):
        side = "right" if self.wall_side > 0 else "left"
        return (f"bay park, wall on the {side}: settle to "
                f"{self.road_middle_mm:.0f}mm and square, reverse to one "
                f"radius ({self.turn_radius_mm():.0f}mm) short of the bay's "
                f"middle, turn {self.turn_in_deg:.0f}deg in, nose to "
                f"{self.nose_stop_mm:.0f}mm; drive {self.speed} / back "
                f"{self.reverse_speed} duty")

    def status_line(self):
        def mm(value):
            return "--" if not _valid(value) else f"{value:.0f}"
        yaw = ("--" if math.isnan(self.wall_angle_deg)
               else f"{self.wall_angle_deg:+.0f}")
        bay = "" if self.bay_mm is None else f" bay={self.bay_mm:.0f}"
        if self.phase == self.TURN_IN:
            leg = f"turned={self.turned_deg:.0f}deg"
        elif self.phase == self.DRIVE_IN:
            leg = f"ahead={mm(self._front_range())}"
        elif self.phase == self.WIGGLE:
            total = len(self.wiggle_steps)
            index = min(self._wiggle_index, total - 1) if total else 0
            step = self.wiggle_steps[index] if total else None
            want = "" if step is None else (
                f" {'back' if step['reverse'] else 'fwd'}"
                f"@{step['steer_deg']:.0f}deg->{step['distance_mm']:.0f}mm")
            leg = f"step={self._wiggle_index + 1}/{total}{want} {self.leg_mm:.0f}mm"
        elif self.phase == self.BACK:
            left = "--" if self.back_target_s is None else \
                f"{self.s_mm - self.back_target_s:.0f}"
            leg = f"togo={left}mm"
        else:
            leg = f"leg={self.leg_mm:.0f}mm"
        return (f"park {self.phase:8} side={mm(self.side_mm)} yaw={yaw}"
                f"{bay} {leg}")

    # ------------------------------------------------------------------
    def update(self, pose, dt, max_steer=40.0):
        """
        One tick. Returns (steering command, speed) - the task writes them.

        `pose` is accepted and ignored: this manoeuvre is measured off the
        lidar, the compass and its own odometry, so it works whether or not
        the localizer is up.
        """
        self.max_steer_seen = max(1.0, abs(max_steer))
        self._last_dt = dt
        self._elapsed += dt
        if self._elapsed >= self.timeout_s and self.phase in self.DRIVING:
            return self._abort(f"timed out after {self._elapsed:.0f}s "
                               f"in {self.phase}")

        self.side_mm = self._side_range()
        self.wall_angle_deg = self._wall_angle()

        if self.phase == self.FIND:
            return self._find(dt)
        if self.phase == self.SETTLE:
            return self._settle(dt)
        if self.phase == self.BACK:
            return self._back(dt)
        if self.phase == self.TURN_IN:
            return self._turn_in(dt)
        if self.phase == self.DRIVE_IN:
            return self._drive_in(dt)
        if self.phase == self.WIGGLE:
            return self._wiggle(dt)
        return (0.0, 0)

    # ------------------------------------------------------------------
    # 1. FIND - drive the wall until the near blade is placed
    # ------------------------------------------------------------------
    def _find(self, dt):
        """Straighten the heading while driving up to the bay wall."""
        self._advance(self.speed, dt)
        self._bank_wall_heading()

        front = self._front_range()
        if _valid(front) and front < self.front_stop_mm:
            return self._abort("a corner arrived before the bay did")

        if (_valid(self.side_mm)
                and self.side_mm < self.road_middle_mm - self.blade_step_mm):
            print(f"Park: bay wall reached at {self.side_mm:.0f}mm; moving to the "
                  f"simple scripted park")
            self._enter(self.SETTLE)
            return (0.0, 0)

        if self.leg_mm >= self.straighten_to_wall_mm:
            print(f"Park: reached the wall approach distance of "
                  f"{self.straighten_to_wall_mm:.0f}mm")
            self._enter(self.SETTLE)
            return (0.0, 0)

        if self.leg_mm >= self.find_max_mm:
            return self._abort(f"no bay wall found in {self.leg_mm:.0f}mm")
        return self._follow_the_wall()

    def _place_bay(self, near_blade_s):
        """
        The bay's middle, one blade (or one dead-reckoned guess) plus the
        field's own known width along - see BayFinder._from_one_blade, whose
        formula this is.

        near_blade_s ITSELF RUNS EARLY, systematically, and blade_lead_mm
        below is where that gets taken back out. get_min_distance reports
        the CLOSEST return over the whole wide sector (SIDE_SECTOR_DEG),
        which is what buys the blade enough dwell time to be confirmed at
        all - but it means the sector starts reporting the blade before the
        axle is actually level with it, from up to roughly a wall-distance's
        worth of tan(half-angle) away. That is a CONSTANT bias in one
        direction (always early, never late), not noise, so it does not
        average out - it has to be added back in, once, as a straight
        offset. A mat run parking short of the bay's true middle is exactly
        this; see parking.blade_lead_mm in config.toml for the actual number
        to nudge.
        """
        step = (self.bay_mm + WALL_THICKNESS_MM) / 2.0
        self.bay_centre_s = near_blade_s + self.blade_lead_mm + step
        print(f"Park: bay ahead - middle at s={self.bay_centre_s:.0f}mm "
              f"({self.bay_mm:.0f}mm wide, blade at {near_blade_s:.0f} "
              f"+{self.blade_lead_mm:.0f} lead)")
        self._enter(self.SETTLE)

    # ------------------------------------------------------------------
    # 3. SETTLE - the one pose the whole open-loop half is measured from
    # ------------------------------------------------------------------
    def _settle(self, dt):
        """Drive forward, straightening the heading before the turn away."""
        self._advance(self.speed, dt)
        if self.leg_mm >= self.forward_distance_mm:
            print(f"Park: forward {self.forward_distance_mm:.0f}mm complete; "
                  f"turning to the opposite side of the bay")
            self._turn_from = self._heading()
            self._enter(self.BACK)
            return (0.0, 0)

        # Active heading correction while walking this extra forward leg. This
        # continuously steers back toward the wall heading instead of merely
        # holding the last command, which is what lets a small yaw error get
        # corrected before the opposite-side turn starts.
        heading = self._heading()
        if heading is not None and self.hold_heading is not None:
            error = angle_difference(heading, self.hold_heading)
            steer = clamp(-self.heading_gain * error,
                          -HEADING_MAX_STEER, HEADING_MAX_STEER)
            steer = clamp(steer, -self.max_steer_seen, self.max_steer_seen)
            return (steer, self._drive(self.speed))
        return (self._hold_heading(), self._drive(self.speed))

    # ------------------------------------------------------------------
    # 4. BACK - straight, on the compass, to the turn-in point
    # ------------------------------------------------------------------
    def _back(self, dt):
        """Turn away from the bay, then straighten the heading as it drives on."""
        self._advance(self.speed, dt)
        if self.leg_mm >= self.forward_to_middle_mm:
            print(f"Park: reached the middle-island approach distance of "
                  f"{self.forward_to_middle_mm:.0f}mm; backing away")
            self._enter(self.TURN_IN)
            return (0.0, 0)

        turn_steer = clamp(-self.wall_side * self.turn_opposite_deg,
                           -self.max_steer_seen, self.max_steer_seen)
        straighten_steer = self._hold_heading()
        if self.leg_mm < self.middle_straighten_after_mm:
            steer = turn_steer
        else:
            steer = straighten_steer
        return (steer, self._drive(self.speed))

    # ------------------------------------------------------------------
    # 5. TURN_IN - full lock toward the wall, counted on the compass
    # ------------------------------------------------------------------
    def _turn_in(self, dt):
        """Reverse away from the middle island until clear of it."""
        self._advance(-self.reverse_speed, dt)
        if self.leg_mm >= self.reverse_away_mm:
            print(f"Park: reversed {self.reverse_away_mm:.0f}mm away from the "
                  f"middle island; turning into the final angle")
            self._enter(self.DRIVE_IN)
            return (0.0, 0)
        return (0.0, self._drive(-self.reverse_speed))

    # ------------------------------------------------------------------
    # 6. DRIVE_IN - straight in until the nose is off the wall
    # ------------------------------------------------------------------
    def _drive_in(self, dt):
        """Turn at an angle for the final exit leg and then straighten out."""
        self._advance(self.speed, dt)
        if self.leg_mm >= self.final_turn_distance_mm:
            print(f"Park: final turn complete after {self.final_turn_distance_mm:.0f}mm; "
                  f"straightening forward")
            self._enter(self.WIGGLE)
            return (0.0, 0)
        steer = clamp(self.wall_side * self.final_turn_deg,
                      -self.max_steer_seen, self.max_steer_seen)
        return (steer, self._drive(self.speed))

    # ------------------------------------------------------------------
    # 7. WIGGLE - optional, off unless wiggle_steps is non-empty. Drives the
    # configured list of (steer_deg, distance_mm, reverse) legs in order,
    # then DONE.
    # ------------------------------------------------------------------
    def _wiggle(self, dt):
        """Drive a short final straight segment to settle the heading."""
        self._advance(self.speed, dt)
        if self.leg_mm >= self.final_straighten_mm:
            print(f"Park: fully straightened after {self.final_straighten_mm:.0f}mm")
            self.phase = self.DONE
            return (0.0, 0)
        return (0.0, self._drive(self.speed))

    # ------------------------------------------------------------------
    # STEERING
    # ------------------------------------------------------------------
    def _follow_the_wall(self):
        """
        Hold road_middle_mm off the outer wall, and hold it parallel.

        FREEZES ON A BLADE. The side beam reads a blade 200mm before it reads
        the wall, and a follow that believes it sees 300mm where it wants 500
        winds on full lock AWAY from the wall - away from the bay it is here
        to park in. So while the beam is on a blade the last good correction
        is held instead. That is not a filter, it is the difference between
        following the wall and following the bay.
        """
        if not self._on_clean_wall():
            return (self._steer, self._drive(self.speed))

        steer = 0.0
        if _valid(self.side_mm):
            steer += self.wall_gain * (self.side_mm - self.road_middle_mm)
        if not math.isnan(self.wall_angle_deg):
            steer += self.angle_gain * self.wall_angle_deg
        steer = clamp(steer, -self.wall_max_steer, self.wall_max_steer)
        # Positive `steer` means "turn toward the wall"; wall_side turns that
        # into a signed command. This is the only mirror in the file.
        wanted = clamp(self.wall_side * steer,
                       -self.max_steer_seen, self.max_steer_seen)
        step = self.steer_slew_deg_s * self._last_dt
        self._steer = clamp(wanted, self._steer - step, self._steer + step)
        return (self._steer, self._drive(self.speed))

    def _hold_heading(self):
        """
        Steering that holds `hold_heading` - the wall's own heading, banked
        while the beam was on clean wall. Written for driving FORWARD; the
        reverse leg negates it.
        """
        heading = self._heading()
        if heading is None or self.hold_heading is None:
            return 0.0
        # + when the nose is clockwise of where it should be, so the
        # correction is negative: steer left to bring it back.
        error = angle_difference(heading, self.hold_heading)
        steer = clamp(-self.heading_gain * error,
                      -HEADING_MAX_STEER, HEADING_MAX_STEER)
        return clamp(steer, -self.max_steer_seen, self.max_steer_seen)

    def _lock_toward_the_wall(self):
        return clamp(self.wall_side * self.max_steer_seen,
                     -self.max_steer_seen, self.max_steer_seen)

    # ------------------------------------------------------------------
    # THE BAY, FROM THE SIDE BEAM
    # ------------------------------------------------------------------
    def _edges(self, dt):
        """
        Watch the side beam for the NEAR blade and log it in axle
        coordinates. Returns True the tick it is confirmed - see _find,
        which places the whole bay from this one reading.

        THE BEAM LEADS THE AXLE. The lidar is lidar_ahead_mm forward of the
        rear axle, so when the beam crosses the edge the axle has not
        reached it yet - the edge is at s + lidar_ahead_mm in axle
        coordinates. That is what near_blade_s is banked in, so bay_centre_s
        comes out as a place the AXLE can be told to go to, which is what
        BACK needs.
        """
        if self.near_blade_s is not None or not _valid(self.side_mm):
            return False
        here = self.s_mm + self.lidar_ahead_mm
        on_blade = self.side_mm < self.road_middle_mm - self.blade_step_mm
        if not on_blade:
            self._pending_edge = None
            return False
        if self._confirm(here):
            self.near_blade_s = here
            print(f"Park: near blade at s={here:.0f}mm")
            return True
        return False

    def _confirm(self, here):
        """
        The blade has to hold for BLADE_CONFIRM_MM before it counts - the
        anti-flicker window a wide sector still needs right at a physical
        blade's edge. See BLADE_CONFIRM_MM's comment for the scripted test
        that sized this against SIDE_SECTOR_DEG's dwell.
        """
        if self._pending_edge is None:
            self._pending_edge = here
            return False
        if here - self._pending_edge >= BLADE_CONFIRM_MM:
            self._pending_edge = None
            return True
        return False

    def _on_clean_wall(self):
        """Is the side beam looking at the outer wall, and not at a blade?"""
        return (_valid(self.side_mm)
                and self.side_mm > self.road_middle_mm - self.blade_step_mm)

    def _bank_wall_heading(self):
        """
        Remember which way the wall runs, averaged, while the beam is clean.

        Only a clean stretch can say. The moment a blade enters the arc the
        fit is still a good line - just the wrong one, square across the road
        - and that is exactly when the straight legs start needing the answer.
        So it is banked here and held.
        """
        if not self._on_clean_wall() or math.isnan(self.wall_angle_deg):
            return
        heading = self._heading()
        if heading is None:
            return
        # The wall's heading is the robot's, undoing however crooked it is.
        #
        # SIGN VERIFIED NUMERICALLY, NOT BY HAND - a synthetic scan (a known
        # straight wall, a robot placed at a KNOWN, deliberately crooked
        # heading) showed the `heading - wall_side*wall_angle_deg` this
        # replaced banking double the true error instead of removing it: a
        # robot 10deg off the wall banked hold_heading 20deg off, not 0.
        # BACK is the only leg that actually SUFFERS for it - it is the only
        # one that steers off hold_heading with the sign that assumes it is
        # correct (see _back's own docstring on why reversing needs the
        # negated correction on TOP of a correct target); DRIVE_IN escaped
        # notice because it holds the heading TURN_IN measured directly on
        # the compass, never this banked value. That is what made it look
        # like "everything was straight, and then only the reverse turned
        # the wrong way" - the target square only became visible once
        # something depended on it.
        wall = heading + self.wall_side * self.wall_angle_deg
        self._heading_samples.append(wall)
        if len(self._heading_samples) > 50:
            self._heading_samples.pop(0)
        # A mean, not the last reading: the fit is worth a couple of degrees
        # tick to tick, and the last tick is the most likely to be poisoned.
        base = self._heading_samples[0]
        offsets = [angle_difference(s, base) for s in self._heading_samples]
        self.hold_heading = (base + sum(offsets) / len(offsets)) % 360.0

    # ------------------------------------------------------------------
    # SENSORS
    # ------------------------------------------------------------------
    def _side_range(self):
        """Closest return in the narrow sector facing the wall, or nan."""
        if self.lidar is None:
            return float("nan")
        half = self.side_sector_deg / 2.0
        bearing = self.side_bearing_deg * self.wall_side
        distance, _ = self.lidar.get_min_distance(bearing - half,
                                                  bearing + half)
        return float("nan") if distance is None else float(distance)

    def _front_range(self):
        if self.lidar is None:
            return float("nan")
        half = FRONT_SECTOR_DEG / 2.0
        distance, _ = self.lidar.get_min_distance(-half, half)
        return float("nan") if distance is None else float(distance)

    def _side_points(self):
        """
        The visible arc of wall as (along, depth) millimetres, mirrored into
        the frame where the wall is on the right.
        """
        try:
            scan = self.lidar.get_scan()
        except (AttributeError, TypeError):
            return []
        if scan is None or len(scan) < 360:
            return []
        first = int(round(self.side_bearing_deg - self.angle_arc_deg))
        last = int(round(min(self.side_bearing_deg + self.angle_arc_deg,
                             USABLE_FOV_DEG)))
        points = []
        for bearing in range(first, last + 1):
            distance = float(scan[int(round(bearing * self.wall_side)) % 360])
            if not _valid(distance):
                continue
            radians = math.radians(bearing)
            points.append((distance * math.cos(radians),
                           distance * math.sin(radians)))
        return points

    def _wall_angle(self):
        """
        Yaw relative to the wall in degrees, + = nose turned AWAY from it, or
        nan when the arc does not look like a wall.

        A LINE FIT, NOT TWO RAYS. Two ranges either side of the perpendicular
        give the angle in one line of trigonometry and give it terribly: a
        centimetre of error across a 20-degree baseline at 300mm is several
        degrees of yaw, and that times the gain is a command that weaves on
        noise alone. Fitting through the whole arc averages them down.
        """
        if self.lidar is None or self.angle_gain <= 0.0:
            return float("nan")
        points = self._side_points()
        if len(points) < self.angle_min_points:
            return float("nan")
        line = self._fit(points)
        if line is None:
            return float("nan")
        # One robust pass: a blade entering the front of the arc would drag
        # the fit round, so throw the outliers out and fit what is left.
        kept = self._without_outliers(points, line)
        if len(kept) < self.angle_min_points:
            return float("nan")
        if len(kept) < len(points):
            line = self._fit(kept)
            if line is None:
                return float("nan")
        yaw = math.degrees(math.atan(line[0]))
        if abs(yaw) > self.angle_max_deg:
            return float("nan")     # a corner or a step, not the wall
        return yaw

    @staticmethod
    def _step_field(step, name, default):
        """
        One field off a wiggle_steps entry, which may be a plain dict (what
        TOML's [[array of tables]] parses into) or any object carrying the
        same field as an attribute - so a caller building steps in Python
        is not forced to build dicts.
        """
        if isinstance(step, dict):
            return step.get(name, default)
        return getattr(step, name, default)

    @staticmethod
    def _fit(points):
        """Least squares depth = slope*along + intercept."""
        n = len(points)
        if n < 2:
            return None
        sx = sum(p[0] for p in points)
        sy = sum(p[1] for p in points)
        sxx = sum(p[0] * p[0] for p in points)
        sxy = sum(p[0] * p[1] for p in points)
        denominator = n * sxx - sx * sx
        if abs(denominator) < 1e-9:
            return None
        slope = (n * sxy - sx * sy) / denominator
        return (slope, (sy - slope * sx) / n)

    @staticmethod
    def _without_outliers(points, line, limit_mm=25.0):
        slope, intercept = line
        return [p for p in points
                if abs(p[1] - (slope * p[0] + intercept)) <= limit_mm]

    def _heading(self):
        if self.compass is None:
            return None
        try:
            return self.compass.heading()
        except Exception:
            return None

    def _turned_since(self, start, dt):
        """
        How far round the robot has come, on the compass if there is one and
        on the arc it must have driven if there is not.
        """
        heading = self._heading()
        if heading is not None and start is not None:
            return abs(angle_difference(heading, start))
        # No compass: the turn is at a known radius, so the arc says the
        # angle. Less good - it believes the wheels - but never nothing.
        return math.degrees(self.leg_mm / max(1.0, self.turn_radius_mm()))

    # ------------------------------------------------------------------
    # ODOMETRY AND PLUMBING
    # ------------------------------------------------------------------
    def _step(self, speed, dt):
        """
        How far this tick moved. Measured against the duty the wheels will
        ACTUALLY be given - see _drive - because a command below the floor is
        driven at the floor, and measuring the command instead is how a leg
        quietly runs 10% long.
        """
        return abs(self._drive(speed)) / 100.0 * self.mm_per_s_at_full * dt

    def _advance(self, speed, dt):
        step = self._step(speed, dt)
        self.s_mm += step if speed >= 0 else -step
        self.leg_mm += step

    def _drive(self, speed):
        """Floor the duty at the one the robot actually moves on."""
        speed = int(speed)
        if speed > 0:
            return max(speed, self.min_move_speed)
        if speed < 0:
            return min(speed, -self.min_move_speed)
        return 0

    def _enter(self, phase):
        self.phase = phase
        self.leg_mm = 0.0
        self._settle_blind_mm = 0.0

    def _abort(self, reason):
        self.phase = self.ABORTED
        self.reason = reason
        print(f"Park aborted: {reason}")
        return (0.0, 0)

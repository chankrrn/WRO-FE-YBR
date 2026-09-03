"""
Parking as a six-state machine driven off four lidar ranges and the camera.

This is a port of the KMIDS Veloz team's RaspberryPi5/src/parking.cpp onto our
hardware. It replaces the geometric bay-finding manoeuvre that used to live in
classes/parking.py, and it is deliberately a straight translation rather than a
reinterpretation: same states, same thresholds, same order of tests.

    SEARCHING      keep driving the lap until the camera holds a bay marker
    APPROACH       drive on alongside the bay until it draws level, then a
                   little further, so the reverse starts from beside it
    REVERSING      back INTO the bay, steering toward the wall side, until
                   the flank is alongside it or the budget runs out
    STRAIGHTENING  keep reversing, steering on the left/right difference, until
                   the two sides agree within STRAIGHT_TOLERANCE_MM
    CENTERING      wheels straight, speed zero
    DONE           stopped

WHAT CHANGED IN THE PORT. The original writes servo angles 40..140 (90 centre)
straight at a Pico over I2C. Our MotorManager takes steering in degrees, minus
for left and plus for right, and update() hands (steer, speed) back to the task
rather than touching the motor - so odometry and the status line keep working.
The angles map one for one onto degrees off centre: their 55/90/125 is our
-35/0/+35, and their 40/140 travel limit is our MAX_STEER_DEG of 50.

The original also keys SEARCHING off red or green camera blobs. On our field the
bay walls are magenta and the red/green blocks are obstacles, so the marker test
here is VisionManager.parking_marks() - the pink bay walls - above the same
minimum area. Watching for red or green instead would start the park at the
first traffic block on the lap.

The lidar watchdog is kept but expressed in our terms: LidarManager already
drops readings older than half a second, so a direction with nothing to say
comes back NaN. Every range NaN at once is the stale-scan case the original
stops for.
"""

import math

# ============================================================================
# The original's constants, unchanged except where the units differ
# ============================================================================
# SPEED IS NOT A PERCENTAGE HERE. The original's 25/18/-18 are percent of full
# throttle over an I2C field clamped to -100..100. Ours goes to analogWrite
# unchanged, so it is a raw PWM duty out of 255, and the robot does not move
# at all below about 55 (see [speed] minimum in the round's config). Ported
# literally, SEARCHING commanded 25 and the robot sat still with the wheels
# not turning, which is exactly what the first mat run did.
#
# So the three speeds come from the config in OUR units instead, and anything
# non-zero is floored at MIN_MOVE_SPEED on the way out. The original's ratios
# are kept for reference only: search was ~1.4x the parking speed.
MIN_MOVE_SPEED = 55

ORIGINAL_PERCENT_SEARCH = 25
ORIGINAL_PERCENT_PARK = 18
ORIGINAL_PERCENT_REVERSE = -18

# Steering in degrees off centre, - left / + right. The original's 40/90/140
# servo range with 55/125 for the two turns.
STEERING_LIMIT = 50.0
STEERING_CENTER = 0.0
STEERING_LEFT = -35.0
STEERING_RIGHT = 35.0

MIN_VALID_MM = 80.0
MAX_VALID_MM = 6000.0

FRONT_STOP_MM = 300.0
BACK_STOP_MM = 250.0
SIDE_CLOSE_MM = 250.0

# How close the bay side may get while backing in before the wheels centre and
# the rest of the leg is driven straight. Below this the swing is finished and
# carrying on would put the flank into a bay wall.
BAY_CLEAR_MM = 150.0

# The original's STRAIGHTENING test: 80mm of left/right difference either way.
STRAIGHT_TOLERANCE_MM = 80.0

# THE AREA IS A RANGE GATE. There is no distance in a ParkingMark - the bearing
# is all the camera gives - so the only thing that says "the bay is HERE" and
# not "the bay is somewhere over there" is how big it looks. A bay wall is
# 200 x 100mm, and on this camera (100deg over 640px) it subtends:
#
#       500mm  10800 px2      1500mm   1200 px2
#       800mm   4200 px2      2000mm    670 px2
#      1000mm   2700 px2      3000mm    300 px2
#      1200mm   1900 px2      4000mm    170 px2
#
# 150 - ObjectSolver's own floor, which is set for "is a bay wall visible at
# all" and not for "am I beside it" - is therefore a bay wall FOUR METRES
# away, which is most of the field's diagonal. A mat run duly triggered on the
# real bay while it was still on the far side of the mat, aligned toward it,
# reversed where it stood and stopped in the left of the field with the bay on
# the bottom edge. Every part of that was the port working correctly on a
# sighting it should never have accepted.
#
# 1200 is about 1.5m head-on, and less than that once the wall is seen at an
# angle and foreshortened - so in practice "within a metre or so, roughly
# facing it", which is the condition the rest of the manoeuvre assumes.
MIN_MARKER_AREA = 1200.0

# And the robot has to actually be running beside a wall. Mid-field both side
# beams read long; beside the outer wall one of them is short. This does not
# discriminate as hard as the area does - the lap runs about 500mm off the
# wall anyway - but it rules out triggering from the middle of the mat.
TRIGGER_SIDE_MM = 900.0

# Which sector of the scan each of the original's four ranges comes from. The
# original got these from its own lidar layer; ours reads whole sectors, so
# each is the closest return over a window centred on that bearing.
FRONT_BEARING_DEG = 0.0
RIGHT_BEARING_DEG = 90.0
BACK_BEARING_DEG = 180.0
LEFT_BEARING_DEG = 270.0
SECTOR_DEG = 20.0

# Every range NaN for this long means the scan has stopped arriving.
LIDAR_TIMEOUT_S = 0.5

# HOW FAR THE REVERSE MAY RUN WITHOUT A REAR RANGE. The original ends both
# REVERSING and STRAIGHTENING on the back distance, because its lidar can see
# behind it. Ours cannot: the C1 stands on the mast at the FRONT of the robot,
# so everything astern of it is the robot's own body, inside the sensor's 50mm
# minimum range and dropped as noise. The back sector reads NaN at every pose,
# on every tick of every mat run so far - which left REVERSING with no exit at
# all, backing up until the timeout.
#
# So the reverse is additionally bounded by odometry. This is a DEPARTURE from
# the original, and an unavoidable one: it is not a tuning choice, it is the
# substitute for a sensor this robot does not have. If the rear ever does read,
# the range test still wins - it is checked first, and this only catches the
# case where the range never arrives.
# HOW FAR THE REVERSE RUNS. A FIXED DISTANCE, and it took two regressions to
# get back to that.
#
# It was briefly derived from the bay-side beam at the start of the leg - "the
# tail may travel as far as the wall is, less a stopping gap". That reasoning
# is wrong, and the mat proved it: by the time the reverse begins the robot is
# already alongside the wall, so that beam reads the width of the gap it is
# standing in, not the distance it has to travel to get in. One run started
# the leg with the bay side at 246mm, took 246 - 150 = 96, clamped it to the
# 80mm floor, and reversed 56mm before declaring itself done. The run everyone
# liked had, by luck, been measuring the FAR beam (1034mm) and so always hit
# the cap - which is to say it was already using a fixed distance and nobody
# noticed.
#
# So: a fixed distance, tuned between the two mat runs that bracket it. The
# tail hit the wall at 711mm; 431mm left the robot half in with its nose still
# out in the road. 480 for the reverse and 120 to square up puts 600 between
# them, nearer the end that costs nothing.
REVERSE_STOP_MM = 150.0        # kept for the status line and config
MIN_REVERSE_MM = 80.0
# 480, NOT 350. Where the bay-side beam reads far - it sees down the road past
# the blades, not the wall - the cap is what binds, and at 350 (+120 for the
# squaring) the reverse ended at 470mm with the robot's back half in the bay
# and its front half still out in the road. The run before that one had no cap
# worth the name and drove the tail into the wall at 711mm. So the useful
# travel is between those two, and 480 + 120 = 600 sits in it with 111mm of
# margin on the end that does damage.
MAX_REVERSE_MM = 480.0

# IN THE BAY. The reverse should end because the robot is IN, not because it
# has run out of budget - the mat run backed 604mm and declared itself parked
# on odometry alone, without anything ever having checked. Inside the bay the
# flank sits beside the outer wall, so the bay-side beam reads short; out in
# the road it reads the width of the road. That step is the test.
#
# Confirmed over several ticks and only after MIN_REVERSE_MM, because that
# beam swings wildly while the robot rotates in - one mat run saw the same
# side go 267, 244, 698, 1120, 455 over five ticks - and a single tick under
# the threshold part-way through the swing is not the bay.
IN_BAY_MM = 180.0
IN_BAY_TICKS = 5
# As a fraction of the reverse budget: the in-bay test cannot end the leg
# before this much of it has been driven.
IN_BAY_AFTER = 0.7

# ONE FRAME IS NOT A BAY. The trigger used to be "a pink blob of 150px exists
# in this frame", and that is an edge, not a state: one false positive
# anywhere on the lap latched the park into ALIGNING and the manoeuvre then
# committed regardless. A mat run did exactly that - aligned, reversed and
# stopped on the wrong side of the field with mark=n on every single line,
# because whatever it triggered on was gone by the next frame. Red and pink
# sit 7 hue units apart (see utils/image_color_utils.py), so a red pillar in
# the wrong light is the obvious candidate.
#
# Held in SECONDS, not ticks. The camera runs at ~15fps against a 50Hz control
# loop, so three consecutive ticks can be the same frame read three times and
# prove nothing.
MARKER_HOLD_S = 0.4

# And losing it un-commits. Whatever ALIGNING is steering at, if the bay is
# not there any more it was not the bay - go back to looking rather than
# reversing into open field. Longer than MARKER_HOLD_S: a mark that flickers
# for one frame behind a pillar should not throw away a real approach.
MARKER_LOST_S = 0.8

# DRIVING ALONGSIDE THE BAY BEFORE REVERSING INTO IT.
#
# The original goes straight from "a marker is visible" to closing on it,
# because its robot starts pointed AT the gap between two markers and only has
# to drive in. Ours meets the bay side-on, halfway down a wall it is driving
# along, so the moment the camera first picks the bay up is the worst possible
# place to reverse from: the slot is somewhere ahead and off to one side, and
# the robot is still pointing down the lap. Every mat run that "parked right
# after seeing the bay" was this.
#
# So APPROACH carries it forward until the bay draws level and then a little
# further, and only then does the reverse start. Level is measured off the
# camera: the mark's bearing grows as the robot comes up on it, and passes
# ABEAM_BEARING_DEG as it goes by. The lens sees +/-50deg, so a bay that is
# genuinely alongside leaves the frame - which counts as level too, and is
# what usually fires first.
ABEAM_BEARING_DEG = 40.0

# How much further to roll once it is level, so the reverse starts from beside
# the far blade rather than from the mouth.
PAST_BAY_MM = 150.0

# If neither the bearing nor the loss of the mark has fired by here, the thing
# that was seen was not a bay - go back to looking rather than reversing into
# open field.
APPROACH_MAX_MM = 1200.0

# HOLDING THE WALL WHILE IT APPROACHES.
#
# The approach used to drive dead straight, on the reasoning that a steering
# correction would move the pose the reverse is measured from. That holds for
# a roll-on of a few centimetres and falls apart for a long one: the track
# CURVES, and a robot going straight leaves it. A mat run with past_bay_mm at
# 800 drove from 296mm off the line to 320mm the other side of it, straight
# through a corner and away from the bay, with the wheels centred the whole
# way - "it doesn't track the bay, it just walks forward".
#
# So the approach holds its distance to the bay-side wall instead, which is
# what keeps it beside the bay however far it rolls. Gently: this is a hold,
# not a hunt, and every degree of it moves where the reverse begins.
APPROACH_WALL_GAIN = 0.05
APPROACH_MAX_STEER = 25.0

# The bay blades stand 200mm proud of the wall, so the side beam DIPS as the
# robot goes by. That dip is the bay, not a drift toward the wall, and
# steering away from it is exactly wrong - so a reading this far below the
# distance being held is ignored and the last correction is kept.
APPROACH_BLADE_MM = 120.0

# The squaring leg reverses too. It gets a SHORT extra allowance, not another
# full budget: by then the robot is already in, and every millimetre of it is
# spent against the 200mm the bay has.
STRAIGHTEN_EXTRA_MM = 120.0

# HOW SHORT A SIDE BEAM HAS TO BE TO BE A BAY WALL AND NOT THE ROAD.
#
# STRAIGHTENING closes the left/right difference, and that test only means
# anything if BOTH beams are looking at the two blades. The original's robot
# finishes between them, so for it they always are. Ours frequently does not:
# the C1 is on the mast at the FRONT, the bay is 200mm deep, and until the
# robot is all the way in those beams are out in the road on both sides.
#
# What the difference then measures is the ROAD's asymmetry - wall on one
# side, open field on the other, hundreds of millimetres apart - and closing
# THAT rotates the robot until it points across the road. Backing up on left
# lock swings the tail left and the nose RIGHT, so with the bay on the right
# and R short, L long, the squaring drives the nose round into the bay: the
# bay ends up in front of the robot instead of beside it, and every degree of
# it is undoing rotation the reverse just paid for. That is the same class of
# fault as the dead rear beam - a test inherited from a sensor geometry this
# robot does not have - and it is handled the same way, by not pretending the
# reading is there.
#
# So both sides have to read like blades before the difference is believed.
# The bay's half width is about 170mm and the body's is 80, which leaves
# roughly 90mm a side when centred, so a blade reads well inside SIDE_CLOSE_MM
# and the road does not. Anything else takes the original's own fallback for
# a side that will not read: stop, and keep the pose the reverse achieved.
SQUARE_BLADE_MM = SIDE_CLOSE_MM

DEFAULT_MM_PER_S_AT_FULL = 390.0

# Nothing in the original bounds the whole manoeuvre; ours does, because the
# round has a time limit and the task retries an aborted park.
DEFAULT_TIMEOUT_S = 60.0


def valid_distance(distance):
    """The original's validity gate: a range outside this is not believed."""
    return (distance is not None
            and not math.isnan(distance)
            and MIN_VALID_MM <= distance <= MAX_VALID_MM)


class Ranges:
    """The four distances the state machine runs on, in mm. NaN for nothing."""

    __slots__ = ("front", "left", "right", "back")

    def __init__(self, front, left, right, back):
        self.front = front
        self.left = left
        self.right = right
        self.back = back

    @property
    def all_missing(self):
        return not any(valid_distance(v)
                       for v in (self.front, self.left, self.right, self.back))


class VelozParking:
    """
    The ported state machine, wearing the interface the final task expects.

    The task builds this with the whole [parking] config section; almost none
    of it applies any more - the original has no wall-following, no bay
    measurement and no compass step - so the settings that have no counterpart
    are accepted and ignored rather than made into an error. The handful that
    do map onto something are honoured: speed, reverse_speed, align_stop_mm,
    side_bearing_deg, side_sector_deg, camera_confirms and timeout_s.

    Never touches the motor: update() returns (steer, speed) and the task
    applies it.
    """

    # APPROACH replaces the original's ALIGNING, which steered toward
    # whichever side had more room until the front closed up. That is the
    # right move for a robot already facing the gap between two markers and
    # the wrong one for a robot driving PAST the bay along a wall - it just
    # wandered across the road and then reversed from wherever it stopped.
    SEARCHING, APPROACH, REVERSING, STRAIGHTENING, CENTERING, DONE, ABORTED = (
        "searching", "approach", "reversing", "straightening", "centering",
        "done", "aborted")
    DRIVING = (SEARCHING, APPROACH, REVERSING, STRAIGHTENING, CENTERING)

    def __init__(self, lidar=None, compass=None, wall_side=1.0,
                 vision=None,
                 camera_confirms=True,
                 search_drives=False,
                 speed=MIN_MOVE_SPEED,
                 search_speed=None,
                 reverse_speed=MIN_MOVE_SPEED,
                 min_move_speed=MIN_MOVE_SPEED,
                 align_stop_mm=FRONT_STOP_MM,
                 side_bearing_deg=RIGHT_BEARING_DEG,
                 side_sector_deg=SECTOR_DEG,
                 min_marker_area=MIN_MARKER_AREA,
                 trigger_side_mm=TRIGGER_SIDE_MM,
                 reverse_stop_mm=REVERSE_STOP_MM,
                 min_reverse_mm=MIN_REVERSE_MM,
                 max_reverse_mm=MAX_REVERSE_MM,
                 straighten_extra_mm=STRAIGHTEN_EXTRA_MM,
                 in_bay_mm=IN_BAY_MM,
                 in_bay_ticks=IN_BAY_TICKS,
                 in_bay_after=IN_BAY_AFTER,
                 marker_hold_s=MARKER_HOLD_S,
                 marker_lost_s=MARKER_LOST_S,
                 abeam_bearing_deg=ABEAM_BEARING_DEG,
                 past_bay_mm=PAST_BAY_MM,
                 approach_max_mm=APPROACH_MAX_MM,
                 approach_wall_gain=APPROACH_WALL_GAIN,
                 approach_max_steer=APPROACH_MAX_STEER,
                 approach_blade_mm=APPROACH_BLADE_MM,
                 heading_ok=None,
                 mm_per_s_at_full=DEFAULT_MM_PER_S_AT_FULL,
                 timeout_s=DEFAULT_TIMEOUT_S,
                 **unused):
        self.lidar = lidar
        self.compass = compass          # kept for the task; unused here
        self.vision = vision
        self.wall_side = float(wall_side)
        self.camera_confirms = bool(camera_confirms)
        # Whether SEARCHING drives itself, as the original does, or leaves the
        # wheels to whatever is running the round. The full round wants the
        # latter - see the `searching` property - but a bench test of the park
        # on its own has no lap to hand them to, so it wants the former.
        self.search_drives = bool(search_drives)

        self.min_move_speed = int(min_move_speed)
        self.speed = int(speed)
        # The original searches faster than it parks, in the same ratio.
        self.search_speed = int(search_speed if search_speed is not None
                                else round(self.speed
                                           * ORIGINAL_PERCENT_SEARCH
                                           / ORIGINAL_PERCENT_PARK))
        self.reverse_speed = -abs(int(reverse_speed))
        # NOT parking.front_stop_mm. That key is 450 and means "the follow
        # runs 450mm off the wall" - the old park's wall-following distance,
        # which is not a stopping distance at all. Fed to this state it ended
        # ALIGNING on the very first tick, at F=428, before the robot had got
        # anywhere near the bay - and the reverse then started from beside the
        # road instead of beside the slot. The original's own number is 300.
        self.align_stop_mm = float(align_stop_mm)
        self.side_sector_deg = float(side_sector_deg)
        self.min_marker_area = float(min_marker_area)
        self.trigger_side_mm = float(trigger_side_mm)
        self.reverse_stop_mm = float(reverse_stop_mm)
        self.min_reverse_mm = float(min_reverse_mm)
        self.max_reverse_mm = float(max_reverse_mm)
        self.straighten_extra_mm = float(straighten_extra_mm)
        self.in_bay_mm = float(in_bay_mm)
        self.in_bay_ticks = int(in_bay_ticks)
        self.in_bay_after = float(in_bay_after)
        self.marker_hold_s = float(marker_hold_s)
        self.marker_lost_s = float(marker_lost_s)
        self.abeam_bearing_deg = float(abeam_bearing_deg)
        self.past_bay_mm = float(past_bay_mm)
        self.approach_max_mm = float(approach_max_mm)
        self.approach_wall_gain = float(approach_wall_gain)
        self.approach_max_steer = float(approach_max_steer)
        self.approach_blade_mm = float(approach_blade_mm)
        # Optional "is the robot in the right sector" test, supplied by the
        # round - see FinalTask._same_sector_as_the_lap_start. None means no
        # such check, which is what a bench run without a lap gets.
        self.heading_ok = heading_ok
        self.mm_per_s_at_full = float(mm_per_s_at_full)
        self.timeout_s = float(timeout_s)

        # The task hands over the whole config section. What is left over is
        # everything the geometric park needed and this one does not; it is
        # recorded so the summary can say so rather than silently dropped.
        self.ignored_settings = sorted(unused)

        self.phase = self.SEARCHING
        self.reason = None
        self.ranges = Ranges(float("nan"), float("nan"),
                             float("nan"), float("nan"))
        self.saw_marker = False
        # How long the bay has been in sight, and how long it has been gone.
        self.marker_held_s = 0.0
        self.marker_lost_for_s = 0.0
        self.give_ups = 0
        # How far the approach has run, and where along it the bay drew level.
        self.approach_mm = 0.0
        self.abeam_at_mm = None
        # The distance to the bay-side wall the approach is holding, and the
        # last correction it made - kept so a blade passing through the beam
        # does not throw the steering.
        self.hold_mm = None
        self._approach_steer = 0.0
        self.max_steer_seen = STEERING_LIMIT
        self.reversed_mm = 0.0
        # Set from the bay-side beam on the first tick of the reverse.
        self.reverse_budget_mm = None
        self.rear_blind = False
        # The squaring leg found the road either side of it rather than the
        # two blades, so it stopped instead of squaring - see SQUARE_BLADE_MM.
        self.squared_blind = False
        self.in_bay = False
        self._in_bay_run = 0
        # Which side the bay is really on - see _bay_side.
        self.bay_side = float(wall_side)
        self._elapsed = 0.0
        self._blind_s = 0.0

    # ------------------------------------------------------------------
    # THE TASK'S INTERFACE
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

    def summary(self):
        side = "right" if self.wall_side > 0 else "left"
        camera = ("camera starts it" if self.camera_confirms
                  else "no camera gate")
        sector = "" if self.heading_ok is None else ", in the start sector only"
        ignored = (f"; {len(self.ignored_settings)} settings from the old "
                   f"park ignored" if self.ignored_settings else "")
        return (f"veloz FSM, wall on the {side}; {camera} over "
                f"{self.min_marker_area:.0f}px (~1.5m) within "
                f"{self.trigger_side_mm:.0f}mm of a wall{sector}, search {self.search_speed} / "
                f"park {self.speed} / back {self.reverse_speed} duty, "
                f"forward to "
                f"{self.align_stop_mm:.0f}mm, back to {BACK_STOP_MM:.0f}mm, "
                f"square within {STRAIGHT_TOLERANCE_MM:.0f}mm{ignored}")

    def status_line(self):
        def mm(value):
            return "--" if not valid_distance(value) else f"{value:.0f}"
        marker = "y" if self.saw_marker else "n"
        if self.give_ups:
            marker += f"({self.give_ups} lost)"
        back = ("blind" if self.rear_blind else mm(self.ranges.back))
        budget = ("" if self.reverse_budget_mm is None
                  else f"/{self.reverse_budget_mm:.0f}")
        if self.phase == self.APPROACH:
            level = ("" if self.abeam_at_mm is None
                     else f" level@{self.abeam_at_mm:.0f}")
            hold = "" if self.hold_mm is None else f" hold={self.hold_mm:.0f}"
            return (f"park {self.phase:13} F={mm(self.ranges.front)} "
                    f"L={mm(self.ranges.left)} R={mm(self.ranges.right)} "
                    f"mark={marker} on={self.approach_mm:.0f}mm{level}{hold}")
        where = " IN" if self.in_bay else ""
        if self.squared_blind:
            where += " nosquare"
        return (f"park {self.phase:13} F={mm(self.ranges.front)} "
                f"L={mm(self.ranges.left)} R={mm(self.ranges.right)} "
                f"B={back} mark={marker} "
                f"back={self.reversed_mm:.0f}{budget}mm{where}")

    # ------------------------------------------------------------------
    def update(self, pose, dt, max_steer=STEERING_LIMIT):
        """
        One tick. `pose` is accepted and ignored - the original has no
        localizer, and nothing here needs one.

        I/O:
            return: (steer_deg, speed), or None once the park has ended
        """
        if self.finished:
            return None

        # THE TIMEOUT IS THE MANOEUVRE'S, NOT THE SEARCH'S. Searching now
        # lasts as long as it takes the lap to bring the bay into view, which
        # can be most of a lap; charging that against a 20s manoeuvre budget
        # aborts the park before it has begun. The round's own time limit is
        # what bounds the search.
        if self.phase != self.SEARCHING:
            self._elapsed += dt
            if self._elapsed > self.timeout_s:
                return self._abort(f"{self.phase} timed out after "
                                   f"{self._elapsed:.1f}s")

        self.max_steer_seen = float(max_steer)
        self.ranges = self._read_ranges()

        # The original's stale-scan watchdog: stop rather than drive blind.
        if self.ranges.all_missing:
            self._blind_s += dt
            if self._blind_s > LIDAR_TIMEOUT_S:
                return self._hold()
        else:
            self._blind_s = 0.0

        self.saw_marker = self._marker_visible()
        if self.saw_marker:
            self.marker_held_s += dt
            self.marker_lost_for_s = 0.0
        else:
            self.marker_held_s = 0.0
            self.marker_lost_for_s += dt

        if self.phase == self.SEARCHING:
            return self._searching()
        if self.phase == self.APPROACH:
            return self._approach(dt)
        if self.phase == self.REVERSING:
            return self._reversing(dt)
        if self.phase == self.STRAIGHTENING:
            return self._straightening(dt)
        return self._centering()

    # ------------------------------------------------------------------
    # THE STATES
    # ------------------------------------------------------------------
    @property
    def searching(self):
        """
        Still looking for the bay, so the LAP should have the wheels.

        The original searches by driving straight ahead: it starts pointed at
        the bay and only has to close on it. Ours starts wherever the last lap
        ended, with the bay somewhere along a wall it is driving beside, so
        straight ahead is into the outer wall - which is what the mat runs
        did, ending jammed against it with the front lidar inside its own
        minimum range and reading nothing at all.

        So SEARCHING does not drive here. The task keeps the round's own path
        following - pure pursuit, the planned line, the pillar dodging - and
        only takes the wheels away when this leaves SEARCHING. See
        FinalTask.parking_command. The state still runs every tick; what it
        does is watch the camera.
        """
        return self.phase == self.SEARCHING and not self.search_drives

    def _searching(self):
        """
        Watch for the bay. The command returned here is what the original
        would have driven, and the task discards it while `searching` - see
        the property above.
        """
        # A HELD sighting, not a single frame - see MARKER_HOLD_S.
        if self.marker_held_s >= self.marker_hold_s:
            self.phase = self.APPROACH
        return self._command(self.search_speed, STEERING_CENTER)

    def _measure_bay_side(self):
        """
        Which side the bay is actually on, from the lidar.

        `wall_side` comes from the lap direction, and on the bench it comes
        from a --wall flag somebody typed. Both can be wrong, and one mat run
        proved it: the robot was started with --wall left and reversed with
        L=978 against R=267, so the wall it was parking against was plainly on
        its right and every steering decision was mirrored.

        The outer wall is the nearest thing to the robot in the plane it is
        driving in, so the shorter of the two side beams is the side it is on.
        Falls back to the declared side when only one of them reads.
        """
        left, right = self.ranges.left, self.ranges.right
        if valid_distance(left) and valid_distance(right):
            self.bay_side = 1.0 if right < left else -1.0
        else:
            self.bay_side = self.wall_side

    def _bay_range(self):
        return self.ranges.right if self.bay_side > 0 else self.ranges.left

    def _far_range(self):
        return self.ranges.left if self.bay_side > 0 else self.ranges.right

    def _approach(self, dt):
        """
        Drive on alongside the bay, and only start reversing once past it.

        This is where the manoeuvre is positioned. The reverse that follows is
        open-loop against a fixed budget, so wherever this stops is the pose
        the whole park is measured from - which is the argument for ending it
        on something that actually means "the bay is beside me" rather than on
        the first tick a marker appeared.

        Three things can end it:

            the bearing swings past abeam_bearing_deg - the bay is drawing
                level, seen directly
            the mark is lost after having been held - it has gone out of the
                lens' +/-50deg, which for a bay the robot is drawing level
                with is the same event, and usually the one that fires
            the front range closes inside align_stop_mm - something is ahead,
                so stop approaching whatever the camera thinks

        Any of them starts a roll-on of past_bay_mm, so the reverse begins
        beside the far blade rather than at the mouth. Running past
        approach_max_mm without any of them means it was never a bay.
        """
        self.approach_mm += self._step_mm(self.speed, dt)

        # NOT A BAY. Same reasoning as the old ALIGNING's un-commit: this is
        # the last state before the reverse, which cannot be undone.
        if self.abeam_at_mm is None and self.approach_mm >= self.approach_max_mm:
            self.give_ups += 1
            self._restart_search()
            return self._command(self.search_speed, STEERING_CENTER)

        self._measure_bay_side()

        if self.abeam_at_mm is None and self._drawing_level():
            self.abeam_at_mm = self.approach_mm

        if (self.abeam_at_mm is not None
                and self.approach_mm - self.abeam_at_mm >= self.past_bay_mm):
            self.phase = self.REVERSING

        return self._command(self.speed, self._hold_the_wall())

    def _hold_the_wall(self):
        """
        Steering that keeps the approach the same distance off the bay-side
        wall as it was when it started.

        Not a wall-follower in the old park's sense - there is no line fit and
        no heading term, just enough correction to stop the robot walking off
        a curving track. The distance it holds is the one it happened to have
        when the approach began, because that is the offset the lap was
        driving at and the one the reverse geometry was tuned around.
        """
        bay_range = self._bay_range()
        if not valid_distance(bay_range):
            return self._approach_steer

        if self.hold_mm is None:
            self.hold_mm = bay_range
            return STEERING_CENTER

        # A blade in the beam, not a drift - see APPROACH_BLADE_MM.
        if bay_range < self.hold_mm - self.approach_blade_mm:
            return self._approach_steer

        error = bay_range - self.hold_mm
        wanted = self.bay_side * self.approach_wall_gain * error
        self._approach_steer = max(-self.approach_max_steer,
                                   min(self.approach_max_steer, wanted))
        return self._approach_steer

    def _drawing_level(self):
        """
        Is the bay coming abeam?

        The bearing passing abeam_bearing_deg says so directly. Losing a mark
        that was being held says the same thing less directly - it has left
        the lens' field - and on a bay the robot is driving past, that is the
        usual way it happens. A front range that has closed up ends the
        approach too, whatever the camera says.
        """
        if valid_distance(self.ranges.front) and self.ranges.front < self.align_stop_mm:
            return True
        bearing = self._marker_bearing()
        if bearing is not None and abs(bearing) >= self.abeam_bearing_deg:
            return True
        return self.marker_lost_for_s >= self.marker_lost_s

    def _restart_search(self):
        self.phase = self.SEARCHING
        self.marker_held_s = 0.0
        self.approach_mm = 0.0
        self.abeam_at_mm = None
        self.hold_mm = None
        self._approach_steer = 0.0
        # The distance to the bay-side wall the approach is holding, and the
        # last correction it made - kept so a blade passing through the beam
        # does not throw the steering.
        self.hold_mm = None
        self._approach_steer = 0.0

    def _reversing(self, dt):
        """
        Back in, steering away from whichever side has closed up.

        Ends on the back range if there is one, and on how far it has backed
        up if there is not - see REVERSE_STOP_MM and _set_budget.
        """
        d = self.ranges
        self.reversed_mm += self._step_mm(self.reverse_speed, dt)
        self._set_budget()

        # STEER TOWARD THE BAY, NOT AWAY FROM THE NEAREST WALL.
        #
        # This is the one place the port could not be kept literal. The
        # original steers away from whichever side has closed inside 250mm,
        # because by the time it reverses it is already nose-in between the
        # two bay walls and the only thing left to do is avoid brushing them.
        # Ours arrives BESIDE the bay, driving along the wall the bay is set
        # into, so the nearest wall IS the bay - and "steer away from the
        # nearest wall" is an instruction to reverse out into the road. That
        # is what the mat run did: outer wall on the right, R=267 and closing,
        # so it steered left and backed away from the slot it was parked
        # beside, exactly as the rule told it to.
        #
        # Reversing into a bay on your right means steering right, the same as
        # it does in a car. `wall_side` is +1 when the outer wall - and so the
        # bay - is on the robot's right, which the task takes from the lap
        # direction, so that is the sign to use.
        toward_bay = STEERING_RIGHT if self.bay_side > 0 else STEERING_LEFT
        bay_range = self._bay_range()
        far_range = self._far_range()

        if valid_distance(bay_range) and bay_range < BAY_CLEAR_MM:
            # Deep enough in that any more swing puts the flank into a blade.
            steering = STEERING_CENTER
        elif valid_distance(far_range) and far_range < SIDE_CLOSE_MM:
            # The original's guard, kept for what it is actually good for:
            # the far side closing means the nose is swinging into the road's
            # other edge, so stop winding on.
            steering = STEERING_CENTER
        else:
            steering = toward_bay

        self._watch_for_the_bay(bay_range)

        if valid_distance(d.back) and d.back < BACK_STOP_MM:
            self.phase = self.STRAIGHTENING
        elif self.in_bay:
            # IN. This is the exit that should fire, and the two below are the
            # backstops for when it does not.
            self.phase = self.STRAIGHTENING
        elif self.reversed_mm >= self._budget():
            self.rear_blind = True
            self.phase = self.STRAIGHTENING
        return self._command(self.reverse_speed, steering)

    def _straightening(self, dt):
        """
        Keep reversing, closing the left/right difference, until the two sides
        agree - or until one of them stops reading, which the original treats
        as good enough to stop on rather than a reason to keep shuffling.

        Bounded by the same odometry as REVERSING, and for the same reason:
        this state reverses too, so with the sides disagreeing and no rear
        range it would otherwise back across the field squaring up.
        """
        d = self.ranges
        self.reversed_mm += self._step_mm(self.reverse_speed, dt)
        if self.reversed_mm >= self._budget() + self.straighten_extra_mm:
            self.rear_blind = True
            self.phase = self.CENTERING
            return self._command(0, STEERING_CENTER)
        steering = STEERING_CENTER

        # BOTH BEAMS ON BLADES, or the difference is the road and not the bay
        # - see SQUARE_BLADE_MM. Out in the road this used to rotate the nose
        # into the wall until the two sides agreed.
        between_blades = (valid_distance(d.left) and valid_distance(d.right)
                          and d.left < SQUARE_BLADE_MM
                          and d.right < SQUARE_BLADE_MM)

        if between_blades:
            difference = d.left - d.right
            if difference > STRAIGHT_TOLERANCE_MM:
                steering = STEERING_LEFT
            elif difference < -STRAIGHT_TOLERANCE_MM:
                steering = STEERING_RIGHT
            else:
                self.phase = self.CENTERING
        else:
            self.squared_blind = True
            self.phase = self.CENTERING

        return self._command(self.reverse_speed, steering)

    def _centering(self):
        """Wheels straight, stopped. The original's CENTERING then DONE."""
        self.phase = self.DONE
        return (STEERING_CENTER, 0)

    # ------------------------------------------------------------------
    # SENSORS
    # ------------------------------------------------------------------
    def _read_ranges(self):
        """The four sector minima the state machine runs on."""
        if self.lidar is None:
            nan = float("nan")
            return Ranges(nan, nan, nan, nan)
        return Ranges(self._sector_min(FRONT_BEARING_DEG),
                      self._sector_min(LEFT_BEARING_DEG),
                      self._sector_min(RIGHT_BEARING_DEG),
                      self._sector_min(BACK_BEARING_DEG))

    def _sector_min(self, bearing_deg):
        half = self.side_sector_deg / 2.0
        distance, _ = self.lidar.get_min_distance(bearing_deg - half,
                                                  bearing_deg + half)
        return float(distance)

    def _watch_for_the_bay(self, bay_range):
        """
        Has the flank come alongside the outer wall - i.e. is the robot in?

        Wants IN_BAY_TICKS in a row, and will not look at all until the
        reverse has covered min_reverse_mm: at the start of the leg the robot
        is still beside the road with the beam pointing straight at the wall
        it is about to back past, which reads short for reasons that have
        nothing to do with being parked.
        """
        # Only near the END of the leg. Beside the wall the bay-side beam is
        # already short for reasons that have nothing to do with being parked
        # - one run began the reverse with both sides under 210mm - so letting
        # this fire early just reproduces the 56mm reverse by another route.
        # It is an overshoot guard, not the primary exit.
        if self.in_bay or self.reversed_mm < self.in_bay_after * self._budget():
            return
        if valid_distance(bay_range) and bay_range < self.in_bay_mm:
            self._in_bay_run += 1
            if self._in_bay_run >= self.in_bay_ticks:
                self.in_bay = True
        else:
            self._in_bay_run = 0

    def _set_budget(self):
        """
        How far the reverse runs: max_reverse_mm, flat.

        Deliberately NOT read off the side beam any more - see the comment on
        REVERSE_STOP_MM for the run where that collapsed the whole leg to
        56mm. There is no measurement available at the start of the reverse
        that says how far the robot has to travel to be in, so this does not
        pretend there is one. The honest version is a distance measured on the
        mat, and the two runs that bracket it are recorded above.
        """
        if self.reverse_budget_mm is None:
            self.reverse_budget_mm = self.max_reverse_mm

    def _budget(self):
        return (self.max_reverse_mm if self.reverse_budget_mm is None
                else self.reverse_budget_mm)

    def _step_mm(self, speed, dt):
        """
        How far this tick moved, from the speed command. The 100 is not a
        percentage - see startup.mm_per_s_at_full, which is measured against
        the same scale the duty is written on.
        """
        return abs(speed) / 100.0 * self.mm_per_s_at_full * dt

    def _marker_bearing(self):
        """
        Bearing of the biggest bay wall in frame, or None if there is none.

        ParkingMark carries a bearing and no distance, so this is the only
        thing the camera can say about WHERE the bay is - and it is enough for
        the one question the approach asks: has it come level yet.
        """
        if self.vision is None:
            return None
        marks = [m for m in self.vision.parking_marks()
                 if m.area_px >= self.min_marker_area]
        if not marks:
            return None
        return max(marks, key=lambda m: m.area_px).bearing_deg

    def _marker_visible(self):
        """
        A bay wall big enough to believe. With the camera gate off, or with no
        vision at all, the park starts immediately - which is what the original
        does when its camera reports nothing but the operator has told it to go.
        """
        if not self.camera_confirms or self.vision is None:
            return True
        # RIGHT SECTOR FIRST. The bay is in the section the round started in,
        # and the field looks the same in all four - so a bay wall seen from
        # the correct distance in the WRONG section is still not the bay to
        # park in. The heading answers that without going near the pose,
        # which matters because the pose is the thing that gets it wrong.
        if self.heading_ok is not None and not self.heading_ok():
            return False
        marks = self.vision.parking_marks()
        if not any(mark.area_px >= self.min_marker_area for mark in marks):
            return False
        # Beside a wall, not out in the middle of the mat.
        closest_side = min([r for r in (self.ranges.left, self.ranges.right)
                            if valid_distance(r)], default=None)
        return closest_side is None or closest_side <= self.trigger_side_mm

    # ------------------------------------------------------------------
    def _command(self, speed, steering):
        """
        Clamp the steering to the servo's travel and to whatever the task
        allows, and floor the speed at the duty the robot actually moves on.

        The floor is not tuning, it is the difference between driving and
        stalling: a duty under MIN_MOVE_SPEED turns the wheels not at all, so
        a state that asks for one never reaches the range that would end it
        and the park sits there until the timeout.
        """
        limit = min(STEERING_LIMIT, abs(self.max_steer_seen))
        speed = int(speed)
        if speed > 0:
            speed = max(speed, self.min_move_speed)
        elif speed < 0:
            speed = min(speed, -self.min_move_speed)
        return (max(-limit, min(limit, steering)), speed)

    def _hold(self):
        return (STEERING_CENTER, 0)

    def _abort(self, reason):
        self.phase = self.ABORTED
        self.reason = reason
        return (STEERING_CENTER, 0)

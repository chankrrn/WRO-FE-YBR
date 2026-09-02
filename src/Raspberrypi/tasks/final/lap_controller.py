"""
Driving a lap by measuring the walls, not by planning a path.

Two numbers steer this robot. `target_outer_mm` is how far from the outer
wall it wants to be, and `turning_front_mm` is how close to the wall ahead
it gets before committing to the corner. Everything else - pillar
avoidance, corner geometry, the racing line - falls out of choosing those
two numbers well. There is no planner, so there is nothing to churn.

    Why this replaces the planner

The old path re-planned about ten times a second, and each run could pick a
different winning candidate than the last, so the pursuit target jumped
between ticks and the robot never committed to one side of a pillar. It
also asked for 515mm of lateral clearance inside a 1000mm lane, so almost
every plan came back "compromised" and was driven anyway.

Here, avoidance is not a manoeuvre. It is picking one of five lane
distances, from a table. A pillar you must pass on the side nearest a wall
gets the hard squeeze (250mm or 760mm); anything else gets the soft one.
The ladder IS the clearance budget, expressed as something the lane can
actually deliver instead of a request it cannot meet.

    Why the corner is where the pillars get hit

Steering lock is fixed, so the arc the robot traces through a corner is
fixed too. That means WHEN the arc starts completely determines where it
ends - and therefore which side of the next segment's first pillar the
robot exits on. Turning 200mm earlier or later is the entire path-planning
system. Choosing that trigger from the NEXT segment's first pillar, rather
than reacting once already in the segment, is the direct fix for exiting a
corner into the centre wall.

    The heading datum cannot drift

Heading error is measured against a quantized cardinal - 0/90/180/270 -
advanced by exactly 90 degrees per corner. It is never integrated from the
steering command, so it cannot accumulate. Everything lateral is measured
against a wall fitted this tick. Neither quantity comes from the localizer.

Ported from the winning team's obstacle_challenge main.cpp. Their constants
are in metres and percent; ours are in millimetres and degrees of steer,
and every conversion is spelled out at its constant.
"""
import math
from collections import namedtuple

from classes.pid import PID
from classes.slot_map import (LANE_MM, Location, Side, next_segment,
                              segment_from_heading)
from utils.angle_utils import angle_difference, clamp

# ============================================================================
# Where to drive in the lane, in mm from the outer wall
# ============================================================================
# Kept as a named tuple as well as loose constants so config.toml can move
# the whole ladder without a twelve-argument constructor, and so a sweep can
# reach it. The loose names stay because the tests and the docstrings above
# read better with them, and because they are the DEFAULT ladder.
Lanes = namedtuple("Lanes", "centre outer1 outer2 inner1 inner2 blind")

LANE_CENTRE_MM = 500.0        # no pillar in play, and throughout every turn
LANE_OUTER1_MM = 430.0        # ease toward the outer wall
LANE_OUTER2_MM = 250.0        # hard squeeze against the outer wall
LANE_INNER1_MM = 620.0        # ease toward the inner wall
LANE_INNER2_MM = 760.0        # hard squeeze against the inner wall

# WHERE TO DRIVE WHEN THE MAP CANNOT SAY. Distinct from centre, because the
# two questions are different: centre is where to be when there is nothing
# to avoid, and this is where to be when there MIGHT be. A pillar stands
# 435mm out (OUTER) or 595mm (INNER) and the body needs 125mm, so the only
# distances that clear BOTH rows are the two hard squeezes, 250 and 760 -
# every softer lane, centre included, is inside one of them.
#
# 760 over 250 because the ways they fail are not equal. 250 leaves 150mm of
# body to the outer wall, and the outer wall is what the crashes are against;
# 760 leaves 180mm to the centre block, which is a large clean fit that the
# resolver never loses. It also keeps the start segment away from the parking
# bay, which stands against the outer wall.
LANE_BLIND_MM = 760.0

LANES = Lanes(LANE_CENTRE_MM, LANE_OUTER1_MM, LANE_OUTER2_MM,
              LANE_INNER1_MM, LANE_INNER2_MM, LANE_BLIND_MM)

# ============================================================================
# When to commit to the corner, in mm from the wall ahead
# ============================================================================
# Front-wall distance over which each slot, in driving order, is the one being
# reacted to. They OVERLAP on purpose (see _choose_lane). The last window's
# low edge doubles as the point where the whole segment has been driven past,
# so an empty row below it is an empty road rather than an unread one.
WINDOWS_MM = ((2000.0, 2900.0), (1500.0, 2700.0), (1000.0, 1800.0))
SEGMENT_SWEPT_MM = WINDOWS_MM[-1][0]

Triggers = namedtuple("Triggers", "default outer1 outer2 inner1 inner2")

PRE_TURN_FRONT_MM = 1200.0    # start looking at the next segment
TURN_FRONT_MM = 780.0         # default trigger, nothing known ahead
TURN_OUTER1_MM = 670.0        # turn later  -> exit nearer the outer wall
TURN_OUTER2_MM = 500.0        # turn much later
TURN_INNER1_MM = 970.0        # turn earlier -> exit nearer the inner wall
TURN_INNER2_MM = 1070.0       # turn earliest

TRIGGERS = Triggers(TURN_FRONT_MM, TURN_OUTER1_MM, TURN_OUTER2_MM,
                    TURN_INNER1_MM, TURN_INNER2_MM)

PRE_TURN_COOLDOWN_S = 4.0

# Give up on a corner whose wall ahead has stopped getting closer for this
# long. 1.8s is the honest run-up at 230mm/s, so 3s is comfortably longer
# than a healthy approach and far shorter than a run.
PRE_TURN_STALL_S = 3.0

# Continuity gate on the lateral ruler - see _outer_mm. At 50Hz and 390mm/s
# flat out the robot covers 8mm a tick, so anything past 120mm is the
# resolver changing its mind about which object it is measuring, not the
# robot moving. Five ticks is 100ms: long enough that a genuine re-acquire
# costs almost nothing, short enough that a real loss is not held stale.
OUTER_JUMP_MM = 120.0
OUTER_JUMP_TICKS = 5
# A cluster of pillars can be fitted as a short wall directly ahead, which
# reads as a corner that is not there. Nothing may re-enter PRE_TURN for
# this long after leaving it; a real corner is at least a segment away.

TURN_EXIT_TOLERANCE_DEG = 30.0
# Deliberately loose. The turn ends 30 degrees early and the wall term
# finishes the alignment smoothly, instead of the heading loop snapping the
# last few degrees with the steering already near full lock.

# ============================================================================
# Gains, converted from theirs
# ============================================================================
WALL_P_DEG_PER_MM = 0.18
# Theirs is 180 degrees of heading offset per METRE of lateral error. Ours
# measures millimetres, so 180/1000. The output is a heading OFFSET, not a
# steer angle - it says "point this much further into the wall", and the
# heading loop below works out what steering that needs.

WALL_P_NO_PILLARS_DEG_PER_MM = 0.30
# Stiffer, for a segment known to hold no pillars: a tighter line and less
# weave when there is nothing to dodge. Restored on PRE_TURN.

WALL_OFFSET_LIMIT_DEG = 90.0
# The wall term may ask for at most a right angle into the wall. Beyond that
# the geometry stops making sense - the robot would be driving across the
# lane rather than along it.

HEADING_P_DEG_PER_DEG = 1.38
# Theirs is 3.0 in units of "percent of full lock per degree of heading
# error", on a +-100 scale. Our motor_manager takes signed degrees, so the
# equivalent is 3.0 * MAX_STEER_DEG / 100 with MAX_STEER_DEG = 46.

MAX_STEER_DEG = 46.0
# Under motor_manager's own MAX_STEER_DEG of 50, leaving the last few
# degrees as headroom for the rate limiter.

NORMAL = "NORMAL"
PRE_TURN = "PRE_TURN"
TURNING = "TURNING"


class LapCommand:
    """What the controller decided this tick."""

    __slots__ = ("steer_deg", "state", "target_outer_mm", "turning_front_mm",
                 "front_mm", "outer_mm", "heading_error_deg", "turn_count",
                 "segment", "wall_lost", "targeted")

    def __init__(self, **kw):
        for name in self.__slots__:
            setattr(self, name, kw.get(name))

    def __repr__(self):
        outer = "none" if self.outer_mm is None else f"{self.outer_mm:.0f}"
        front = "none" if self.front_mm is None else f"{self.front_mm:.0f}"
        return (f"LapCommand({self.state} steer={self.steer_deg:+.1f} "
                f"outer={outer}/{self.target_outer_mm:.0f} front={front} "
                f"seg{self.segment} turns={self.turn_count})")


# ============================================================================
# THE TWO TABLES
# ============================================================================
def _steer_toward_outer(color_is_green, clockwise):
    """
    Does this pillar push the robot toward the outer wall or the inner one?

    The two tables below share this selector, which is the only place the
    colour rule enters. Read as a statement of fact about the tuned tables:
    clockwise-and-green behaves like counter-clockwise-and-red, and the
    other two pair up the same way. The physical rule it encodes is which
    side of the pillar the robot must pass on; the table is taken from a
    configuration measured to work, and the sign is confirmed on the bench
    rather than argued from the rulebook.
    """
    return color_is_green == clockwise


def lane_for(color_is_green, side_is_inner, clockwise, lanes=LANES):
    """How far from the outer wall to drive, given the pillar in play."""
    if _steer_toward_outer(color_is_green, clockwise):
        return lanes.outer1 if side_is_inner else lanes.outer2
    return lanes.inner2 if side_is_inner else lanes.inner1


def turn_trigger_for(color_is_green, side_is_inner, clockwise,
                     triggers=TRIGGERS):
    """
    How close to the wall ahead to commit, given the NEXT segment's first
    pillar.

    Same shape as the lane table, and for the same reason: exiting the
    corner nearer the outer wall means turning later, so a pillar that
    would demand a small target_outer_mm demands a small trigger too.
    """
    if _steer_toward_outer(color_is_green, clockwise):
        return triggers.outer1 if side_is_inner else triggers.outer2
    return triggers.inner2 if side_is_inner else triggers.inner1


# ============================================================================
# THE CONTROLLER
# ============================================================================
class LapController:
    """
    NORMAL -> PRE_TURN -> TURNING -> NORMAL, once per corner.

    Pure decision-making: no hardware, no clock of its own, no I/O. It is
    handed measurements and returns a steer angle, which is what lets the
    whole state machine be exercised offline.
    """

    def __init__(self, clockwise, slot_map, start_segment=0,
                 max_steer_deg=MAX_STEER_DEG, steer_sign=1.0,
                 wall_p=WALL_P_DEG_PER_MM, heading_p=HEADING_P_DEG_PER_DEG,
                 lanes=LANES, triggers=TRIGGERS,
                 pre_turn_front_mm=PRE_TURN_FRONT_MM,
                 pre_turn_cooldown_s=PRE_TURN_COOLDOWN_S,
                 turn_exit_tolerance_deg=TURN_EXIT_TOLERANCE_DEG,
                 no_pillars_wall_p=WALL_P_NO_PILLARS_DEG_PER_MM):
        self.clockwise = bool(clockwise)
        self.slots = slot_map
        self.max_steer_deg = float(max_steer_deg)

        # The two ladders and the three thresholds that depend on the
        # chassis rather than the field. Held per-instance so config.toml
        # can move them and test_driving --sweep can walk them; the module
        # constants above remain the values every one of them defaults to.
        self.lanes = Lanes(*(float(v) for v in lanes))
        self.triggers = Triggers(*(float(v) for v in triggers))
        self.pre_turn_front_mm = float(pre_turn_front_mm)
        self.pre_turn_cooldown_s = float(pre_turn_cooldown_s)
        self.turn_exit_tolerance_deg = float(turn_exit_tolerance_deg)
        self.no_pillars_wall_p = float(no_pillars_wall_p)

        # +1 when a positive command steers the same way a positive heading
        # error needs. Bench-verified per the plan, not assumed - getting it
        # backwards turns the lateral loop into a divergent one.
        self.steer_sign = float(steer_sign)

        self.wall_p = float(wall_p)
        self.wall_pid = PID(self.wall_p,
                            output_min=-WALL_OFFSET_LIMIT_DEG,
                            output_max=WALL_OFFSET_LIMIT_DEG)
        self.heading_pid = PID(float(heading_p),
                               output_min=-self.max_steer_deg,
                               output_max=self.max_steer_deg)
        self.wall_pid.set_active(True)
        self.heading_pid.set_active(True)

        self.state = NORMAL
        self.heading_direction_deg = (int(start_segment) % 4) * 90.0
        self.target_outer_mm = self.lanes.centre
        self.turning_front_mm = self.triggers.default
        self.turn_count = 0
        self._last_pre_turn_at = None
        self._pre_turn_since = None   # when the run-up last made progress
        self._pre_turn_best = None    # closest the wall ahead has come
        self._stiffened = False
        self._targeted = None
        self._outer_held = None       # last accepted lateral reading, mm
        self._outer_doubts = 0        # consecutive ticks disagreeing with it

    # ========================================================================
    # MEASUREMENT
    # ========================================================================
    @property
    def segment(self):
        return segment_from_heading(self.heading_direction_deg)

    def _front_mm(self, walls):
        """
        Distance to the wall ahead, or None.

        Falls back to the wall BEHIND because the front wall is genuinely
        invisible for the first half of a segment - it is 3m away and past
        the lidar's useful range for a fitted line. 3000 is the field size.
        """
        if walls.front is not None:
            return walls.front.perpendicular_distance()
        if walls.back is not None:
            return 3000.0 - walls.back.perpendicular_distance()
        return None

    def _outer_mm(self, walls):
        """
        Distance to the outer wall, or None. Clockwise puts it left.

        Gated for continuity, because the resolver answers "the nearest thing
        on my left" and more than one thing qualifies. The lane is a metre
        wide with the outer wall down one side and the centre block down the
        other, so whenever the two land in the same bin - which a corner
        exit, still crabbed, readily arranges - the block is the nearer and
        `nearest` hands it over. The controller then holds station off the
        BLOCK, and since it believes that face is the outer wall it drives a
        tight square around the middle of the field. That is the crash into
        the centre wall, and nothing about the reading itself looks wrong:
        it is a clean fit at a plausible distance.

        What gives it away is the jump. Between ticks the robot moves about
        5mm, so the outer wall moves about 5mm; the block arriving instead
        of the wall moves it by hundreds. A reading that far from the last
        one is not this wall getting closer, it is a different wall. Hold
        the old value and make the new one prove itself over several ticks
        first - if the lock really was lost, it re-acquires in 100ms.
        """
        outer = walls.left if self.clockwise else walls.right
        inner = walls.right if self.clockwise else walls.left
        if outer is not None:
            measured = outer.perpendicular_distance()
        elif inner is not None:
            measured = LANE_MM - inner.perpendicular_distance()
        else:
            measured = None

        # Mid-corner the outer wall legitimately changes identity, so there
        # is nothing to be continuous with. Forget and re-acquire on exit.
        if self.state == TURNING or measured is None or self._outer_held is None:
            self._outer_held = measured
            self._outer_doubts = 0
            return measured

        if abs(measured - self._outer_held) <= OUTER_JUMP_MM:
            self._outer_held = measured
            self._outer_doubts = 0
            return measured

        self._outer_doubts += 1
        if self._outer_doubts >= OUTER_JUMP_TICKS:
            self._outer_held = measured
            self._outer_doubts = 0
        return self._outer_held

    # ========================================================================
    # CONTROL
    # ========================================================================
    def update(self, dt, walls, heading_deg, now):
        """
        One tick.

        I/O:
            dt: seconds since the last call
            walls: a wall_sense.ResolvedWalls measured this tick
            heading_deg: absolute heading from the IMU, degrees
            now: monotonic seconds, for the pre-turn cooldown only
            return: a LapCommand
        """
        front_mm = self._front_mm(walls)
        outer_mm = self._outer_mm(walls)

        # Chained transitions resolve within one tick - leaving PRE_TURN can
        # immediately satisfy TURNING's condition. Bounded so a bad
        # measurement cannot spin here.
        for _ in range(4):
            if self.state == NORMAL:
                changed = self._normal(front_mm, now)
            elif self.state == PRE_TURN:
                changed = self._pre_turn(front_mm, now)
            else:
                changed = self._turning(heading_deg, now)
            if not changed:
                break

        steer, heading_error = self._steer(dt, outer_mm, heading_deg)
        return LapCommand(
            steer_deg=steer, state=self.state,
            target_outer_mm=self.target_outer_mm,
            turning_front_mm=self.turning_front_mm,
            front_mm=front_mm, outer_mm=outer_mm,
            heading_error_deg=heading_error, turn_count=self.turn_count,
            segment=self.segment, wall_lost=outer_mm is None,
            targeted=self._targeted)

    def _steer(self, dt, outer_mm, heading_deg):
        """
        The cascade: lateral error -> heading offset -> heading error -> steer.

        The wall loop does not produce steering. It produces a heading it
        would rather be pointing, and the heading loop - which is measured
        against a datum that cannot drift - works out the steering. That
        indirection is what keeps a lateral correction from turning into a
        permanent heading bias.
        """
        heading_error = angle_difference(self.heading_direction_deg, heading_deg)

        if outer_mm is not None and self.wall_pid.active:
            offset = self.wall_pid.update(outer_mm - self.target_outer_mm, dt)
            # Being too far from the outer wall must turn the robot toward
            # it, and which way that is depends on which side it is on.
            heading_error -= offset if self.clockwise else -offset
            # AND WRAP AGAIN. angle_difference returns the short way round,
            # but the offset is added after it and can push the sum back out
            # past +-180 - the offset clamps at +-90, so any heading error
            # beyond about 90deg is exposed. Past 180 the SIGN FLIPS, and the
            # heading loop then drives full lock the long way round: a 148deg
            # error reads as -212 and steers away from the correction instead
            # of into it. That is a crash, not a wobble - measured, the robot
            # held -46deg of lock for three seconds straight into the outer
            # wall. Normally invisible because both terms are small on a
            # straight, and because the wall term is off through TURNING,
            # which is the other place the error is large.
            heading_error = (heading_error + 180.0) % 360.0 - 180.0
        elif outer_mm is None:
            # No wall this tick: hold heading rather than steer on a stale
            # lateral error. Discard, do not guess.
            self.wall_pid.reset()

        steer = self.heading_pid.update(heading_error, dt)
        return clamp(self.steer_sign * steer,
                     -self.max_steer_deg, self.max_steer_deg), heading_error

    # ========================================================================
    # STATES
    # ========================================================================
    def _normal(self, front_mm, now):
        """Hold the lane, choosing it from whichever pillar is in play."""
        self._choose_lane(front_mm)

        if front_mm is None or front_mm > self.pre_turn_front_mm:
            return False
        if self._last_pre_turn_at is not None \
                and now - self._last_pre_turn_at < self.pre_turn_cooldown_s:
            return False

        if self._stiffened:
            self.wall_pid.set_gains(self.wall_p, 0.0, 0.0)
            self._stiffened = False
        self.state = PRE_TURN
        self._last_pre_turn_at = now
        # Fresh run-up: the stall watchdog starts from this approach, not
        # from whatever the previous corner left behind.
        self._pre_turn_since = now
        self._pre_turn_best = front_mm
        return True

    def _choose_lane(self, front_mm):
        """
        Which pillar am I reacting to, and where does it put me?

        The three windows OVERLAP on purpose. A pillar's influence fades in
        while the previous one's is still fading out, so a segment with
        several pillars produces one continuous weave instead of three
        discrete lurches. Later windows win, so the nearer pillar takes
        priority where they overlap.

        NO PILLAR IN A WINDOW MEANS HOLD THE LANE, NOT RETURN TO CENTRE.
        The setpoint is a member that persists; only a targeted pillar, or
        the corner, ever moves it. That is deliberate and it is what the
        ported code does - `targetOuterWallDistance_` is assigned in exactly
        two places, the pillar table and the top of TURNING.

        It matters because of where the pillars physically are. Lane centre
        passes an OUTER pillar (435mm out) with 65mm to spare and an INNER
        one (595mm) with 95mm, and the body needs 125mm of either: THE
        MIDDLE OF THE LANE IS THE ONE PLACE THAT HITS EVERYTHING. Snapping
        back to 500 the moment a window closes therefore aims the robot at
        the pillar it is still driving past - the C window shuts at 1000mm
        of front wall and the C pillar stands at 975mm, so the snap lands
        alongside it. Holding the dodge costs nothing: TURNING resets the
        setpoint every corner, so a held lane can never outlive its segment.

        HOLDING IS ONLY RIGHT ONCE THERE IS SOMETHING TO HOLD. The lane is
        reset to centre at the top of every TURNING, so a segment entered
        with nothing committed in it holds CENTRE - the one distance that
        clears neither pillar row - for as long as the map stays quiet, and
        the map is quiet exactly when the robot has not seen far enough into
        the segment yet. That is not the sticky behaviour above failing, it
        is the sticky behaviour being asked a question it cannot answer, and
        it cost a whole run: entering a segment blind at 500mm and driving
        straight into the pillar that was standing there.
        `self.lanes.blind` is the answer to "I do not know" - see its
        definition for why 760 and not 250.

        This is where we part company with the code this was ported from,
        which does snap to centre. It can afford to: at 520-780mm/s its car
        crosses a segment in under two seconds and its slot table is
        populated from the previous lap, so "nothing committed here" is
        nearly always genuine emptiness rather than ignorance. At our
        230mm/s the ignorant window is most of the first lap.
        """
        self._targeted = None
        starting = self.turn_count == 0

        slots = self._segment_order()
        if all(s is None for s in slots):
            # An empty row means one of two opposite things and the front
            # ruler is what tells them apart. Below the last window's low
            # edge the whole segment has been driven past, so empty means
            # EMPTY: nothing to dodge, stiffen the wall term for a straighter
            # line. Above it the segment is simply unread - the slot map
            # commits on three unanimous votes and has not had them yet -
            # and stiffening there would only hold a guess more firmly.
            #
            # The start segment is never "read", because its OUTER slots are
            # vetoed for the whole run: our parking bay is magenta and reads
            # as red, so an outer pillar there could never be believed even
            # if it were seen. It stays blind to the end of the segment.
            swept = (not starting and front_mm is not None
                     and front_mm <= SEGMENT_SWEPT_MM)
            if swept:
                if not self._stiffened:
                    self.wall_pid.set_gains(self.no_pillars_wall_p, 0.0, 0.0)
                    self._stiffened = True
            else:
                self.target_outer_mm = self.lanes.blind
            return

        if front_mm is None:
            # Slots in this row, but no ruler to say which one is alongside.
            # Hold: the lane being held was chosen for a pillar this segment
            # really has, which is a better answer than the blind lane. The
            # front ruler dropping out is usually a tick or two, and swinging
            # from 250 to 760 to cover a transient is a 510mm lateral move
            # made while passing the very pillar the 250 was clearing.
            return

        target = None
        for slot, (low, high) in zip(slots, WINDOWS_MM):
            if slot is not None and low < front_mm <= high:
                target = slot
        if target is None:
            # Between windows, with known pillars either side of the gap.
            # Hold - this is the sticky case the docstring is about, and
            # the lane being held was chosen to clear the pillar we are
            # still driving past. Except before the FIRST window opens,
            # where there is nothing yet to have been chosen by.
            if front_mm > WINDOWS_MM[0][1]:
                self.target_outer_mm = self.lanes.blind
            return

        self._targeted = target
        self.target_outer_mm = lane_for(target.color.name == "GREEN",
                                        target.side is Side.INNER,
                                        self.clockwise, self.lanes)

    def _segment_order(self):
        """This segment's slots, in the order they are driven past."""
        order = (Location.A, Location.B, Location.C) if self.clockwise \
            else (Location.C, Location.B, Location.A)
        return [self.slots.get(self.segment, loc) for loc in order]

    def _pre_turn(self, front_mm, now):
        """
        Pick the turn trigger from the NEXT segment's first pillar.

        This is the lookahead. By the time the corner is entered the exit
        side is already decided, so the robot never has to correct its way
        out of one - which is what drives it into the centre wall.

        PRE_TURN IS OTHERWISE A TRAP, and the escape below is ours, not the
        original's. Its only exit is the front wall coming close enough, and
        `_choose_lane` runs from NORMAL only - so a run-up that stalls freezes
        the lane as well as the state, and the robot holds whatever setpoint
        it happened to have while pillar avoidance is switched off. That is
        not hypothetical: it is a whole run, holding 500mm off the wall to
        within +-15mm, shuttling against a pillar for 165 of 200 seconds.
        The original is safe without this because its car covers the 1200mm
        run-up in about half a second; ours takes 1.8s at 230mm/s and can
        take forever if something is in the way. If the wall ahead has not
        come closer in PRE_TURN_STALL_S, the corner was not real or cannot
        be reached: go back to NORMAL, where the lane can move again.
        """
        self.turning_front_mm = self._trigger_for_next()

        if front_mm is not None:
            if self._pre_turn_best is None or front_mm < self._pre_turn_best - 1.0:
                self._pre_turn_best = front_mm
                self._pre_turn_since = now

        if front_mm is None or front_mm > self.turning_front_mm:
            if self._pre_turn_since is not None \
                    and now - self._pre_turn_since > PRE_TURN_STALL_S:
                self.state = NORMAL
                self._pre_turn_since = None
                self._pre_turn_best = None
                # Re-arm the debounce so it does not walk straight back in
                # on the same reading that got it here.
                self._last_pre_turn_at = now
                return True
            return False

        self._pre_turn_since = None
        self._pre_turn_best = None

        # Advance the datum by exactly 90 degrees. This is the only place
        # heading_direction_deg ever changes, which is why it cannot drift.
        step = 90.0 if self.clockwise else -90.0
        self.heading_direction_deg = (self.heading_direction_deg + step) % 360.0
        self.state = TURNING
        return True

    def _trigger_for_next(self):
        """Prefer the next segment's first pillar; fall back to its middle."""
        nxt = next_segment(self.segment, self.clockwise)
        first = Location.A if self.clockwise else Location.C
        slot = self.slots.get(nxt, first) or self.slots.get(nxt, Location.B)
        if slot is None:
            return self.triggers.default
        return turn_trigger_for(slot.color.name == "GREEN",
                                slot.side is Side.INNER, self.clockwise,
                                self.triggers)

    def _turning(self, heading_deg, now):
        """
        Arc round with the lateral loop switched off.

        Mid-corner there is no stable "outer wall" to measure against - the
        one being left and the one being joined are both in view and the
        classifier is entitled to disagree between ticks. A live lateral
        loop would fight the turn, so it is disabled and its history flushed
        on the way back in.
        """
        self.target_outer_mm = self.lanes.centre
        self.wall_pid.set_active(False)

        if abs(angle_difference(self.heading_direction_deg,
                                heading_deg)) > self.turn_exit_tolerance_deg:
            return False

        self.turn_count += 1
        self.state = NORMAL
        self.wall_pid.set_active(True)
        # RE-ARM THE DEBOUNCE FROM HERE, not from where the corner began.
        # Coming out of a turn the robot is still up to TURN_EXIT_TOLERANCE
        # off the new datum, and at that crab angle the classifier can put
        # the wall we just left in the `front` bin - a short read that walks
        # straight into PRE_TURN and takes the same corner twice.
        #
        # Stamping at PRE_TURN entry instead is what the original did, and it
        # worked there because their car covers the 1200mm lookahead in about
        # a second. Ours takes three and a half, so the whole cooldown was
        # spent before the turn had even started. A duration ported from
        # another chassis is only a duration; what it was protecting is the
        # few seconds AFTER a corner.
        self._last_pre_turn_at = now
        return True

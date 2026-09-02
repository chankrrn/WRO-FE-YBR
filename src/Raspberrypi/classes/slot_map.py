"""
Where the pillars are, remembered as a lattice instead of a point cloud.

A pillar is only ever in one of twelve places on this field: four segments
by three positions along each. Which side of the lane it stands on is a
property OF that place, not a thirteenth degree of freedom. So instead of
carrying a continuous (x, y) estimate that drifts, this snaps every
observation onto the lattice and remembers the cell.

Snapping is lossless here. It would be lossy if a pillar could stand
anywhere; it cannot, so the lattice is a prior that happens to be exactly
true. What it buys is immunity: a 60mm error in the pillar's position does
not move it to a different cell, whereas it does move a metric estimate.

    Reading a cell address off the walls

Position is measured against the fitted walls, never against the localizer.
Inside the current lane, distance from the FRONT wall says how far along
the segment a pillar is, and distance from the OUTER wall says which side.
Past 900mm from the outer wall the pillar is not in this lane at all - it
is around the corner - and the two axes TRANSPOSE: the current outer wall
becomes the along-track ruler for the next segment, and the current front
wall decides that pillar's side. That transposition is what lets one
measurement frame address two segments at once, which is what the corner
lookahead needs.

    Discard, do not guess

Every bin has a dead-band around it and a reading that lands in one is
THROWN AWAY rather than rounded to the nearer bin. A cell we have not
learned yet is a state the controller handles - it drives the blind lane,
which clears both pillar rows. A cell we have learned WRONGLY sends it into
a pillar. The asymmetry is the whole design: this map would rather know
nothing than know something false.

A cell commits to whichever answer leads its running tally, once that
answer has both enough observations and enough of a margin over the
runner-up. It can be revised later by weightier evidence. See _vote().
"""
import math
from enum import Enum

from utils.enums import Color

# ============================================================================
# The lattice
# ============================================================================
FIELD_MM = 3000.0      # matches field_map.FIELD_SIZE_MM
LANE_MM = 1000.0       # (3000 - 1000) / 2, the driving lane width

# Along-track bins, as distance from the wall AHEAD. Slot centres sit at
# roughly 1000/1500/2000mm; the gaps between bins (1150-1350 and 1650-1850)
# are dead-bands, and a reading landing in one is discarded.
ALONG_BINS = (
    (800.0, 1150.0),      # nearest the wall ahead
    (1350.0, 1650.0),     # mid
    (1850.0, 2150.0),     # furthest from the wall ahead
)

# The pillar is in the NEXT segment once it is this far from the outer wall.
NEXT_SEGMENT_MM = 900.0

# Which side of the lane. The boundary is lane centre with a 40mm dead-band.
# That looks tight next to the 200mm along-track dead-bands, but pillars
# stand at roughly 250mm or 750mm from the outer wall, so a reading has to
# be 230mm wrong to cross it - far more headroom than the number suggests.
SIDE_OUTER_MAX_MM = 480.0
SIDE_INNER_MIN_MM = 520.0

# ============================================================================
# Trusting an observation
# ============================================================================
VOTES_TO_COMMIT = 3
# Observations a cell's leading answer needs before it may be believed. NOT
# consecutive - a running tally, see _vote() for why three-in-a-row turned
# out to be no filter at all.
#
# Still three, and that is a measured result rather than an inherited one.
# Swept over 3/6/10/16 with everything else fixed (8 placements, 4 pillars),
# and the trade runs the other way from intuition:
#
#   votes   clean runs   correct side   worst clearance   median clearance
#     3        3/8          30/32           -3.8cm             +4.2cm
#     6        1/8          30/32           -7.2cm             +4.3cm
#    10        0/8          31/32           -6.7cm             +2.9cm
#    16        0/8          31/32           -6.1cm             +1.9cm
#
# Waiting longer buys a third of a pillar of accuracy - 30/32 against 31/32,
# which is one placement and inside the noise - and pays for it with every
# other column. Three is the only value that finishes clean runs at all, and
# it has the best worst case by 2.3cm.
#
# THE MEDIAN COLUMN IS THE ONE THAT EXPLAINS IT. Clearance gets steadily
# WORSE as the threshold rises, +4.2cm down to +1.9cm, because a cell that
# has not committed cannot steer: the robot drives the blind lane until the
# tally is satisfied, and every extra vote is more metres spent going past
# pillars on a lane chosen without them. The accuracy the higher rows gain is
# accuracy about pillars that have already been driven past.
#
# Measured after the veto and mirror fixes. An earlier sweep here recorded
# 3/6/12/24 and a different story; it was taken while _vetoed() was throwing
# away most of the evidence and the filter was on the field's mirror
# solution, so it was measuring a broken map and is not comparable.

COMMIT_MARGIN = 2.0
# ...and the leader must also have this many times the runner-up. The count
# alone says "I have seen this a lot"; the margin says "and I have not been
# seeing something else just as often", which is the half that catches a
# cell whose geometry is genuinely being misread. A cell that stays split
# stays uncommitted and reads as None, which the lap controller handles as
# "unknown" and drives the blind lane for.
#
# IT DOES NOTHING MEASURABLE IN SIMULATION, AND THAT IS WORTH SAYING OUT
# LOUD. Swept 1.0/1.5/2.0/3.0, where 1.0 is the margin switched off
# entirely (8 placements, 4 pillars):
#
#   margin   clean runs   correct side   worst clearance   median clearance
#     1.0       3/8          30/32           -3.8cm             +3.9cm
#     1.5       3/8          30/32           -3.8cm             +4.1cm
#     2.0       3/8          30/32           -3.8cm             +4.2cm
#     3.0       3/8          30/32           -3.8cm             +4.2cm
#
# Four identical rows. Once the cell actually receives its evidence the
# leader wins by hundreds of votes to single figures, so a rule about the
# leader beating the runner-up by 2x never gets to bind on anything.
#
# An earlier sweep recorded here claimed the opposite - that switching the
# margin off cost 6cm of rms and a pillar. That was measured while _vetoed()
# was discarding most of the observations, which left cells deciding on a
# handful of readings where a 2x margin is the difference between 8-to-3 and
# 3-to-1. Fixing the veto removed the conditions the margin was earning its
# keep under. The number stayed; the reason for it did not.
#
# Kept at 2.0 anyway, as insurance rather than as tuning. The sim's side
# classifier is cleaner than the real one will be - real detections bring
# lighting, motion blur and a camera-to-lidar association that degrades under
# yaw - and this is the guard that turns a genuinely split cell into an
# honest None instead of a coin flip. It costs nothing when it does not
# bind, which on this evidence is always. If it ever starts to matter, that
# is a signal about the classifier, not about this constant.

VETOED = "vetoed"
# The tally entry for a reading the parking bay could have faked - see
# _vetoed(). It is COUNTED like any other answer and can never be COMMITTED,
# and both halves matter.
#
# Dropping such a reading on the floor, which is what this used to do, is not
# the same as not having seen it. A start-segment cell holding a real pillar
# still gets read a couple of thousand times; if the believable readings are
# discarded silently, the handful of misclassifications among them are the
# only evidence the cell ever receives, and a unanimous tally of noise commits
# with full confidence. Measured: an OUTER pillar seen 2477 times as OUTER and
# 12 times as INNER committed INNER, because every OUTER reading was vetoed
# and 8 INNER ones were not. The robot then held lane 430 into a pillar
# standing at 435mm and stalled against it for half the run.
#
# Counting them fixes that without pretending to know anything: the vetoed
# entry usually leads its cell, the leader is unbelievable, so the cell stays
# uncommitted and reads as None - which the lap controller drives the blind
# lane for. "I am not allowed to trust what I can see here" and "there is
# nothing here" produce the same safe behaviour, which is the correct outcome
# for both.

MAX_HEADING_RATE_DEG_S = 20.0
# Camera and lidar are not sampled at the same instant, so under yaw the two
# disagree about where a pillar is by (rate * skew). At 20 deg/s and 30ms of
# skew that is already 0.6 degrees; faster than this and the association in
# pillar_range starts pairing the wrong things. Detection is suppressed
# rather than filtered - a turn lasts under a second and the pillars are
# still there afterwards.


class Location(Enum):
    """
    Position along a segment - but WHICH end A is depends on the direction.

    Clockwise, A is ~2m from the wall ahead and C is ~1m, so A is entered
    first. Counter-clockwise the letters run the other way and C is entered
    first. That is not a tidy naming scheme, it is the one the ported code
    uses (combined_processor.cpp: "front (reverse)"), and both halves of the
    pair - classify() and _segment_order() - flip together, so the letters
    stay consistent with each other whichever way the lap runs.
    """
    A = 0
    B = 1
    C = 2


class Side(Enum):
    """Which wall the pillar stands nearer."""
    OUTER = 0
    INNER = 1


class Slot:
    """One committed cell of the lattice."""

    __slots__ = ("segment", "location", "side", "color", "votes")

    def __init__(self, segment, location, side, color, votes=0):
        self.segment = int(segment)
        self.location = location
        self.side = side
        self.color = color
        self.votes = int(votes)

    def __repr__(self):
        return (f"Slot(seg{self.segment} {self.location.name} "
                f"{self.side.name} {self.color})")


# ============================================================================
# SEGMENTS
# ============================================================================
def segment_from_heading(heading_deg):
    """Cardinal segment index: 0 = North, 1 = East, 2 = South, 3 = West."""
    return int(((heading_deg % 360.0) + 45.0) % 360.0 // 90.0)


def heading_for_segment(segment):
    """The cardinal heading a segment is driven along, in degrees."""
    return (int(segment) % 4) * 90.0


def next_segment(segment, clockwise):
    """The segment entered after this one."""
    return (int(segment) + (1 if clockwise else -1)) % 4


# Where a slot physically is, for the inverse below. Along-track values are
# the centres of ALONG_BINS. The lateral pair is not measured off a field -
# it is the only place the lane table can mean, worked backwards. A pillar
# needs robot half-width 100 + pillar half 25 = 125mm of gap to pass. An
# OUTER pillar is passed at 250mm one way round and 620mm the other, so it
# must stand between 375 and 495 - call it 435. An INNER pillar is passed at
# 430 or 760, so it stands between 555 and 635 - call it 595.
#
# Worth noticing what falls out: lane centre, 500mm, clears an OUTER pillar
# by 65mm and therefore does NOT clear it. Driving the middle is only safe
# where there is nothing to drive past.
SLOT_ALONG_MM = (975.0, 1500.0, 2000.0)
SLOT_OUTER_MM = 435.0
SLOT_INNER_MM = 595.0


def slot_position(segment, location, side, clockwise):
    """
    Field coordinates of a lattice cell, in mm. The inverse of classify().

    Lives here rather than in the simulator so that the two definitions
    cannot drift apart: if ALONG_BINS moves, the place the test harness
    stands a pillar moves with it.

    The derivation is two plane equations. With `f` the unit heading of the
    segment and `o` the unit normal pointing at the OUTER wall, the wall
    ahead is the plane p.f = 1500 and the outer wall is p.o = 1500. A cell
    sits `along` from the first and `lateral` from the second, and because
    f and o are orthogonal that pins it exactly:

        p = (1500 - along) * f  +  (1500 - lateral) * o
    """
    heading = math.radians(heading_for_segment(segment))
    fx, fy = math.sin(heading), math.cos(heading)
    # Outer wall is on the robot's left going clockwise - the same identity
    # _walls_for uses, so the two stay consistent by construction.
    ox, oy = (-fy, fx) if clockwise else (fy, -fx)

    # A is entered first and C last, so which of them stands NEAREST the wall
    # ahead depends on which way the lap runs - exactly the flip classify()
    # applies when it turns a bin index back into a Location. Reading
    # SLOT_ALONG_MM at location.value regardless would make this the inverse
    # of classify() for counter-clockwise only, and its mirror image for
    # clockwise: still one of the 24 real sites, but not the one asked for.
    index = (2 - location.value) if clockwise else location.value
    along = SLOT_ALONG_MM[index]
    lateral = SLOT_OUTER_MM if side is Side.OUTER else SLOT_INNER_MM
    half_field = FIELD_MM / 2.0
    return ((half_field - along) * fx + (half_field - lateral) * ox,
            (half_field - along) * fy + (half_field - lateral) * oy)


# ============================================================================
# CLASSIFICATION
# ============================================================================
def _bin_index(value, bins):
    """Which bin `value` falls in, or None if it is in a dead-band."""
    for i, (low, high) in enumerate(bins):
        if low < value < high:
            return i
    return None


def _walls_for(walls, clockwise):
    """
    (outer, inner) wall segments for the direction of travel.

    Clockwise puts the outer wall on the robot's LEFT. That is the same
    identity the park drives on - see FinalTask, "wall on the right is
    counter-clockwise" - so the two agree by construction.
    """
    if clockwise:
        return walls.left, walls.right
    return walls.right, walls.left


def distances(x, y, walls, clockwise):
    """
    (along, lateral) for a point, in mm, or (nan, nan) if unmeasurable.

    `along` is distance to the wall ahead, `lateral` to the outer wall.
    Either can be derived from the opposite wall when its own is not in
    view, which is common: the front wall vanishes for the first half of a
    segment, and the outer wall is occluded whenever a pillar is beside it.
    """
    front, back = walls.front, walls.back
    if front is not None:
        along = front.perpendicular_distance(x, y)
    elif back is not None:
        along = FIELD_MM - back.perpendicular_distance(x, y)
    else:
        along = float("nan")

    outer, inner = _walls_for(walls, clockwise)
    if outer is not None:
        lateral = outer.perpendicular_distance(x, y)
    elif inner is not None:
        lateral = LANE_MM - inner.perpendicular_distance(x, y)
    else:
        lateral = float("nan")
    return along, lateral


def classify(x, y, walls, segment, clockwise):
    """
    Which lattice cell a robot-frame point falls in.

    I/O:
        x, y: the pillar's centre in the lidar frame, mm
        walls: a wall_sense.ResolvedWalls from this same tick
        segment: the segment the robot is currently driving
        clockwise: direction of travel
        return: (segment, Location, Side), or None if the point lands in a
            dead-band, outside every bin, or the walls needed are missing.

    A None here is a normal, frequent outcome and means "not yet", not
    "error". Roughly a third of the lane area is dead-band by design.
    """
    along, lateral = distances(x, y, walls, clockwise)
    if math.isnan(along) or math.isnan(lateral):
        return None

    if lateral < NEXT_SEGMENT_MM:
        # In the lane we are driving: front wall measures along-track.
        seg = int(segment)
        along_value, side_value = along, lateral
    else:
        # Around the corner. The axes swap: how far the pillar is from OUR
        # outer wall is how far along the NEXT segment it sits, and how far
        # it is from OUR front wall decides its side of that lane.
        seg = next_segment(segment, clockwise)
        along_value, side_value = lateral, along

    index = _bin_index(along_value, ALONG_BINS)
    if index is None:
        return None

    # Bins are ordered by distance from the wall ahead. Driving clockwise the
    # nearest bin is the LAST position reached (C); counter-clockwise the
    # segment is traversed the other way and it is the first (A).
    if lateral < NEXT_SEGMENT_MM:
        order = (Location.C, Location.B, Location.A) if clockwise \
            else (Location.A, Location.B, Location.C)
    else:
        # Measured from the outer wall, the next segment reads front-first.
        order = (Location.A, Location.B, Location.C) if clockwise \
            else (Location.C, Location.B, Location.A)
    location = order[index]

    if side_value < SIDE_OUTER_MAX_MM:
        side = Side.OUTER
    elif side_value > SIDE_INNER_MIN_MM:
        side = Side.INNER
    else:
        return None
    return seg, location, side


# ============================================================================
# THE MAP
# ============================================================================
class SlotMap:
    """
    Write-once memory of the lattice.

    Cells are keyed by (segment, Location) - side and colour are what the
    cell HOLDS, matching the winning team's map. That means the lattice
    cannot represent two pillars sharing one along-track position on
    opposite sides of the lane; neither can theirs, and the field does not
    place them that way.
    """

    def __init__(self, start_segment=None, veto_start_outer=True,
                 votes_to_commit=VOTES_TO_COMMIT,
                 max_heading_rate_deg_s=MAX_HEADING_RATE_DEG_S,
                 commit_margin=COMMIT_MARGIN):
        self._slots = {}          # (segment, Location) -> Slot, the leader
        self._votes = {}          # (segment, Location) -> {(side, color): n}
        self.start_segment = start_segment
        self.veto_start_outer = bool(veto_start_outer)
        self.suppressed = False   # was the last observe() gated out?
        # Per-instance so config.toml owns them. Raising the vote count buys
        # certainty with delay; lowering the rate gate buys observations
        # mid-corner at the price of worse camera/lidar pairing.
        self.votes_to_commit = int(votes_to_commit)
        self.max_heading_rate_deg_s = float(max_heading_rate_deg_s)
        self.commit_margin = float(commit_margin)

    # ========================================================================
    # WRITING
    # ========================================================================
    def observe(self, fixes, walls, segment, clockwise, heading_rate_deg_s=0.0):
        """
        Fold one tick of pillar fixes into the map.

        I/O:
            fixes: pillar_range.PillarFix objects from this tick
            walls: the ResolvedWalls measured on the same tick
            segment: segment currently being driven
            clockwise: direction of travel
            heading_rate_deg_s: |yaw rate|; detection is suppressed above
                MAX_HEADING_RATE_DEG_S
            return: list of cells committed by THIS call (usually empty)

        Committing is deliberately rare. Most ticks either confirm what is
        already known or fall in a dead-band and do nothing.
        """
        self.suppressed = abs(heading_rate_deg_s) > self.max_heading_rate_deg_s
        if self.suppressed or walls is None:
            # Add nothing, and keep what is already counted. The streak
            # version cleared the tally here, which was right for it - a gap
            # breaks a run of consecutive readings - but wrong for a running
            # one, where it would throw the whole map away at every corner.
            return []

        seen = {}
        for fix in fixes:
            if not fix.trusted:
                # Camera-only range is too coarse for a 200mm dead-band; let
                # it inform steering, never the map.
                continue
            x, y = fix.position()
            cell = classify(x, y, walls, segment, clockwise)
            if cell is None:
                continue
            seg, location, side = cell
            # Vetoed readings are recorded, not dropped - a cell whose
            # believable evidence is being refused must not then be decided
            # by the unbelievable remainder. See VETOED.
            seen[(seg, location)] = VETOED \
                if self._vetoed(seg, side, fix.color) else (side, fix.color)

        return self._vote(seen)

    def _vetoed(self, segment, side, color):
        """
        A RED reading in a start-segment OUTER slot could be our own bay.

        Our parking bay is marked in magenta and sits against the outer wall
        of the start segment. Magenta under track lighting lands close enough
        to red that a bay marker can read as a red pillar in an OUTER slot -
        which would push the robot toward the inner wall for no reason.

        COLOUR IS PART OF THE TEST, and leaving it out was a bug. The thing
        being guarded against is a magenta marker read as red; it cannot be
        read as green, so a green pillar in a start-segment OUTER slot is
        exactly as believable as one anywhere else. Vetoing it threw away the
        only evidence that could have identified a pillar the robot then
        drove into - see VETOED for the measurement. Narrowing this to RED
        gives up nothing: a red pillar there is still refused, which is the
        case the veto was written for.
        """
        return (self.veto_start_outer
                and self.start_segment is not None
                and segment == self.start_segment
                and side is Side.OUTER
                and color is Color.RED)

    def _vote(self, seen):
        """
        Weigh the evidence for each cell and commit the leader.

        THIS USED TO BE "VOTES_TO_COMMIT IDENTICAL READINGS IN A ROW, THEN
        WRITE-ONCE", ported faithfully, and it is wrong for us. Consecutive
        unanimity sounds like a strong filter and is not one at 50Hz: three
        in a row spans 60ms, and every glitch worth worrying about - one
        occluded frame, one bad camera/lidar pairing - lasts longer than
        that, so it produces its three in a row by construction. Measured on
        one segment, the tally at the moment of commit was 22 OUTER against
        3 INNER and the cell committed INNER, because the three INNERs
        happened to land together. Write-once then discarded the 2524 OUTER
        confirmations that followed. The robot spent the rest of the run
        driving the inner-pillar lane, 430mm out, into a pillar standing at
        435mm - stalled against it for two thirds of the round.

        So: keep a running tally per cell and let the leader stand, provided
        it has both VOTES_TO_COMMIT observations and COMMIT_MARGIN times the
        runner-up. That is strictly more noise-immune than three-in-a-row -
        a burst has to beat the accumulated weight of everything before it,
        not merely arrive together - while remaining able to correct itself.

        It also drops write-once, deliberately. Freezing bought stability for
        a pillar already passed, which no longer steers anything; what it
        cost was the ability to fix a bad guess about a pillar still ahead,
        which is the only kind that matters.

        A cell whose evidence is genuinely split stays uncommitted and reads
        as None, which is the discard-don't-guess habit this map is built on.
        """
        committed = []
        for key, answer in seen.items():
            tally = self._votes.setdefault(key, {})
            tally[answer] = tally.get(answer, 0) + 1

            ranked = sorted(tally.items(), key=lambda kv: kv[1], reverse=True)
            leader, best = ranked[0]
            runner_up = ranked[1][1] if len(ranked) > 1 else 0
            if best < self.votes_to_commit \
                    or best < self.commit_margin * runner_up:
                continue
            if leader is VETOED:
                # The best-supported answer is one we are not allowed to
                # believe, so the cell reads unknown rather than falling
                # through to whatever came second.
                #
                # WITHDRAWING, not merely declining to commit. A cell can
                # commit early off a dozen readings and only later collect
                # the evidence that this site cannot be trusted at all -
                # measured, one committed INNER/RED on a 12-to-6 tally and
                # finished the run at 24 against 2708 vetoed, still holding
                # the guess. Leaving it standing is the failure this whole
                # module is written against: knowing something false.
                #
                # The two thresholds give this hysteresis rather than a
                # flutter. Committing needs the answer to beat VETOED by the
                # margin; withdrawing needs VETOED to beat it by the same
                # margin; in between neither fires and the cell keeps what
                # it has.
                self._slots.pop(key, None)
                continue
            best_side, best_color = leader

            was = self._slots.get(key)
            if was is not None and was.side is best_side \
                    and was.color == best_color:
                continue                       # already says this
            slot = Slot(key[0], key[1], best_side, best_color, best)
            self._slots[key] = slot
            committed.append(slot)
        return committed

    # ========================================================================
    # READING
    # ========================================================================
    def get(self, segment, location):
        """The committed slot at this cell, or None."""
        return self._slots.get((int(segment) % 4, location))

    def first_in(self, segment, clockwise):
        """
        The first pillar encountered on entering a segment.

        This is what the corner lookahead asks for: the turn-entry distance
        is chosen from the next segment's first pillar, so the arc ends with
        the robot already on the correct side of it.
        """
        order = (Location.A, Location.B, Location.C) if clockwise \
            else (Location.C, Location.B, Location.A)
        for location in order:
            slot = self.get(segment, location)
            if slot is not None:
                return slot
        return None

    def segment_slots(self, segment):
        """Every committed slot in a segment, in A/B/C order."""
        return [s for s in (self.get(segment, loc) for loc in Location)
                if s is not None]

    def is_empty(self, segment):
        """
        True when a segment is known to hold no pillars.

        Only meaningful once the segment has been driven; before that it is
        indistinguishable from "not looked at yet", which is why the caller
        gates the adaptive-gain trick on lap count rather than on this alone.
        """
        return not self.segment_slots(segment)

    @property
    def committed(self):
        return dict(self._slots)

    def __len__(self):
        return len(self._slots)

    def __repr__(self):
        cells = ", ".join(str(s) for s in self._slots.values())
        return f"SlotMap({len(self._slots)}/12: {cells})"

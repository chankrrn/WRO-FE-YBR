"""
Finding the parking bay and getting the robot into it.

The bay is two magenta walls stuck to the outer wall of the start section,
each 200mm long, 10mm thick and 100mm tall, with a gap between them of about
1.25 robot lengths. The robot has to end up between them, parallel to the
outer wall, touching neither.

WHY THIS IS NOT A PARALLEL PARK
-------------------------------
The textbook two-arc parallel park cannot be driven here, and it fails for a
different reason at each of the two plausible steering calibrations - so the
manoeuvre below deliberately does not depend on which one is true:

    lock 70deg -> rear-axle radius 60mm. Two arcs move the robot sideways by
        at most 2R = 120mm, but it has to come in about 190mm: it cannot drive
        past the bay closer than ~260mm from the outer wall without clipping
        the 200mm wall tips, and it has to end up near 110mm.

    lock 48.5deg -> radius 146mm. Now there is enough sideways travel, but the
        nose corner sweeps a 233mm radius on the closing arc and clips the
        front wall inside a 300mm bay.

So the robot backs in STEEPLY and then squares up inside the bay. Three
numbers shape everything that follows, for a 240 x 120mm robot:

    along-wall footprint   240cos(t) + 120sin(t), peaking at 268mm at t=26.6deg
                           - against a 300mm bay that is 16mm of slack per end,
                           which is why the squaring phase closes the loop on
                           lidar ranges to the walls and not on the pose
    depth footprint        240sin(t) + 120cos(t), under the 200mm the bay is
                           deep only for t below about 21deg - hence a final
                           heading tolerance of roughly +/-10deg
    half-diagonal          134mm, against 100mm of half-depth: at a steep angle
                           the robot's centre has to sit further out than it
                           will finish, and settle in as it straightens

The last one is why squaring and settling are one closed-loop phase with a
clearance guard every tick rather than a scripted sequence of arcs.
"""
import math

from classes.robot_geometry import DEFAULT_LIDAR_OFFSET_MM, LIDAR_AHEAD_MM, to_field
from utils.angle_utils import angle_difference, clamp, normalize_angle

# ============================================================================
# The bay, as the rules build it
# ============================================================================
WALL_LENGTH_MM = 200.0      # how far each wall sticks out from the outer wall
WALL_THICKNESS_MM = 10.0    # its extent ALONG the outer wall
WALL_HEIGHT_MM = 100.0      # low enough that a high lidar mount misses it
NOMINAL_BAY_MM = 340.0      # clear gap between the two inner faces, MEASURED
YAW_ARM_MM = 200.0          # what a radian of yaw is worth, as a length

# Which way is "out from the outer wall", and which axis the walls run along,
# for each of the four sections. (axis, sign) where axis 0 is x and 1 is y:
# `wall_axis` is the coordinate the wall sits at, `along_axis` the one it
# slides along. sign is +1 when the wall is at +outer.
_SECTIONS = {
    "south": (1, -1),
    "north": (1, +1),
    "east": (0, +1),
    "west": (0, -1),
}


def section_of(x, y, field_map):
    """
    Which edge section a field point is in, or None in a corner cell.

    Same four cells FieldMap.start_zones() names, asked the other way round -
    given a point, which one is it in.
    """
    for name, low, high in field_map.start_zones():
        if low[0] <= x <= high[0] and low[1] <= y <= high[1]:
            return name
    return None


def wall_rects(section, center_along_mm, field_map, bay_mm=NOMINAL_BAY_MM,
               length_mm=WALL_LENGTH_MM, thickness_mm=WALL_THICKNESS_MM):
    """
    The two walls of a bay, as axis-aligned boxes ready for
    FieldMap.add_obstacle().

    I/O:
        section: "south" / "east" / "north" / "west"
        center_along_mm: middle of the bay, on whichever axis that section
                         runs along
        bay_mm: clear gap between the walls' inner faces
        return: [((x_min, y_min), (x_max, y_max)), ...] - two of them
    """
    wall_axis, sign = _SECTIONS[section]
    outer = field_map.outer
    # From the bay centre out to each wall's MIDDLE: half the gap plus half a
    # wall, so that `bay_mm` ends up being the clear space between the faces.
    offset = bay_mm / 2.0 + thickness_mm / 2.0

    rects = []
    for direction in (-1.0, +1.0):
        along = center_along_mm + direction * offset
        low, high = [0.0, 0.0], [0.0, 0.0]
        # Across the wall's thickness, along the section's axis.
        along_axis = 1 - wall_axis
        low[along_axis] = along - thickness_mm / 2.0
        high[along_axis] = along + thickness_mm / 2.0
        # And from the outer wall inward by its length.
        if sign > 0:
            low[wall_axis] = outer - length_mm
            high[wall_axis] = outer
        else:
            low[wall_axis] = -outer
            high[wall_axis] = -outer + length_mm
        rects.append((tuple(low), tuple(high)))
    return rects


def bay_pose(section, center_along_mm, field_map, depth_mm):
    """
    Where the robot's centre should sit to be parked, and facing which way.

    `depth_mm` is how far the centre stands off the outer wall. The heading is
    the one that lies ALONG the outer wall; the other way round is equally
    parked, so callers compare modulo 180.

    I/O:
        return: (x_mm, y_mm, heading_deg)
    """
    wall_axis, sign = _SECTIONS[section]
    outer = field_map.outer
    point = [0.0, 0.0]
    point[1 - wall_axis] = center_along_mm
    point[wall_axis] = sign * (outer - depth_mm)
    # Headings are clockwise from +Y: a wall running along X is driven at 90,
    # one running along Y at 0.
    heading = 90.0 if wall_axis == 1 else 0.0
    return point[0], point[1], heading


def bay_interior(section, center_along_mm, field_map, bay_mm=NOMINAL_BAY_MM,
                 length_mm=WALL_LENGTH_MM):
    """
    The parking space itself as an axis-aligned box: between the two walls'
    inner faces, and from the outer wall out to the walls' tips.

    This is what "parked" means - the whole chassis inside this box.

    I/O:
        return: ((x_min, y_min), (x_max, y_max))
    """
    wall_axis, sign = _SECTIONS[section]
    outer = field_map.outer
    along_axis = 1 - wall_axis
    low, high = [0.0, 0.0], [0.0, 0.0]
    low[along_axis] = center_along_mm - bay_mm / 2.0
    high[along_axis] = center_along_mm + bay_mm / 2.0
    if sign > 0:
        low[wall_axis] = outer - length_mm
        high[wall_axis] = outer
    else:
        low[wall_axis] = -outer
        high[wall_axis] = -outer + length_mm
    return tuple(low), tuple(high)


def along_axis_of(section):
    """Which coordinate index (0=x, 1=y) the bay slides along in that section."""
    wall_axis, _ = _SECTIONS[section]
    return 1 - wall_axis


def wall_heading_of(section):
    """Heading, degrees CW from +Y, that runs ALONG that section's outer wall."""
    wall_axis, _ = _SECTIONS[section]
    return 90.0 if wall_axis == 1 else 0.0


def nearest_outer_wall(x, y, field_map):
    """
    Which outer wall a point is closest to, always - even in a corner cell.

    section_of() answers a different question: which of the four LEGAL START
    cells a point is in, and it returns None in the corners. That is right for
    "where may the robot be placed" and wrong for "which wall is this bay
    stuck to", because a bay 800mm along the wall is still on that wall while
    sitting in a cell section_of calls nothing at all.

    I/O:
        return: "south" / "east" / "north" / "west"
    """
    outer = field_map.outer
    return min((("north", outer - y), ("south", y + outer),
                ("east", outer - x), ("west", x + outer)),
               key=lambda pair: pair[1])[0]


def travel_direction_beside_wall(section, heading_deg, min_alignment=0.5):
    """
    Which way round the lap a robot parked against this section's outer wall
    is pointing: +1 counter-clockwise, -1 clockwise (RacingLine's convention).

    The bay is stuck to the OUTER wall, and the robot parks parallel to it -
    so which side of the robot that wall lies on settles the lap direction on
    its own, with no reference to the racing line at all. Going
    counter-clockwise the centre block is on the left and the outer wall is on
    the right; clockwise it is the other way round. That is the same identity
    ParkingController drives on, where `wall_side` is +1 exactly when
    `direction` is +1 - see FinalTask._start_parking.

    Worth having as its own answer because the general one is weaker here.
    RacingLine.direction_for picks whichever direction needs the smaller turn,
    which is right to within 90 degrees of heading error and is the only thing
    available for a robot set down in the open. A robot in a bay knows more
    than that: it knows it is parallel to a wall it can identify, and the
    answer is then exact to within 90 degrees of yaw rather than 90 degrees of
    everything.

    ONLY VALID PARALLEL TO THE WALL, which is how a robot parks in a bay this
    shape - and the reading degrades smoothly rather than suddenly, so it is
    checked rather than assumed. `right . normal` is +/-1 with the robot
    square to the wall and 0 with it nose-on, and a robot anywhere near
    nose-on has no answer here at all: the wall is neither side of it, and
    which sign comes out is decided by a degree or two of yaw. Under
    `min_alignment` this returns None and the caller falls back rather than
    acting on a coin flip.

    I/O:
        section: which of the four outer walls - section_of() or
                 nearest_outer_wall()
        heading_deg: the robot's heading, degrees clockwise from +Y
        min_alignment: how square to the wall the robot has to be, as
                 |cos| of the angle off parallel. 0.5 is 60 degrees.
        return: (direction, alignment) with direction +1 or -1, or
                (None, alignment) when the wall is unknown or the robot is
                too far off parallel to say
    """
    if section not in _SECTIONS:
        return None, 0.0
    wall_axis, sign = _SECTIONS[section]
    # The robot's right, as a unit vector: heading+90 in a convention where
    # forward is (sin, cos).
    heading = math.radians(heading_deg)
    right = (math.cos(heading), -math.sin(heading))
    # Outward normal of that wall: +sign along the axis the wall sits on.
    normal = [0.0, 0.0]
    normal[wall_axis] = float(sign)
    reach = right[0] * normal[0] + right[1] * normal[1]
    if abs(reach) < float(min_alignment):
        return None, abs(reach)
    return (1 if reach > 0.0 else -1), abs(reach)


# ============================================================================
# Finding the bay
# ============================================================================
# How far apart two returns can be, along the wall, and still be the same
# blade. A blade is 10mm thick, so this is almost entirely lidar noise.
CLUSTER_GAP_MM = 45.0
# Votes are pooled into bins this wide before two of them are called a bay.
VOTE_BIN_MM = 25.0
MIN_CLUSTER_POINTS = 3
# Ignore returns beyond this - the far side of the field is not the bay.
MAX_DETECT_RANGE_MM = 2500.0
MIN_DETECT_RANGE_MM = 120.0
# The robot's own chassis blocks everything past about +-120 degrees, which is
# why NavigationManager crops the scan to DEFAULT_FOV_DEG. Anything asked for
# beyond this comes back as the body itself, at roughly its own radius - a
# short, confident, meaningless range. Stop a couple of degrees inside it.
USABLE_FOV_DEG = 118.0


class BayFinder:
    """
    Watches the outer wall of the start section for the two blades, and keeps
    watching until two of them agree.

    Run it on EVERY lap, not just the last one. The bay does not move, the
    robot goes past it three times, and the start point inside the section is
    arbitrary - so accumulating across laps turns the parking lap into "drive
    to a known pose" instead of "hunt for the bay while the clock runs out".

    What a blade looks like to a lidar: a 10mm-thick, 200mm-long fin standing
    out perpendicular to the outer wall. From anywhere along the track the
    beams strike its long face, so the returns land in field coordinates as a
    line of points at a nearly CONSTANT position along the wall, spread out
    across the depth axis. That is the signature clustered for here - tight
    along the wall, deep away from it - and it is why the outer wall itself
    (depth ~0) and the centre block (wrong section) do not look like one.
    """

    def __init__(self, field_map, section, min_depth_mm=120.0,
                 min_gap_mm=250.0, max_gap_mm=400.0, min_scans=3,
                 single_scans=6, lidar_offset_mm=DEFAULT_LIDAR_OFFSET_MM):
        """
        I/O:
            lidar_offset_mm: (forward, right) of the lidar from the rear axle,
                             which is the point `observe`'s pose describes.
                             The returns are projected from the LIDAR, so
                             pass NavigationManager's own offset here or the
                             blades land 15cm along the wall from where they
                             are.
        """
        self.map = field_map
        self.section = section
        self.lidar_offset_mm = tuple(float(v) for v in lidar_offset_mm)
        self.min_depth_mm = float(min_depth_mm)
        self.min_gap_mm = float(min_gap_mm)
        self.max_gap_mm = float(max_gap_mm)
        self.min_scans = int(min_scans)
        self.single_scans = int(single_scans)

        self.scans = 0
        self.blades = 0          # clusters accepted, for the status line
        self._votes = {}         # bin index -> [count, sum of positions]
        self.bay = None          # (section, centre along the wall) once found
        self.bay_mm = NOMINAL_BAY_MM
        self.from_single_blade = False
        # Which way along the wall the robot is travelling, +1 or -1 in the
        # same coordinate `along` is measured in. Needed only to place a bay
        # from ONE blade: the bay is then half a bay AHEAD of it. The task
        # sets this each tick - see FinalTask._look_for_bay.
        self.travel_sign = None

        

    # ------------------------------------------------------------------
    def observe(self, pose, scan):
        """
        Folds one lidar scan into the evidence. Cheap enough to call every
        tick; it is all numpy over 360 points.

        I/O:
            pose: Pose from NavigationManager - a bad one is skipped, because
                  a blade placed through a wrong pose is worse than no blade
            scan: 360-element array of mm, index = bearing, nan where empty
            return: the bay as (section, centre_mm) once known, else None
        """
        if self.bay is not None or self.section is None:
            return self.bay
        if pose is None or not pose.is_reliable:
            return None

        along, depth = self._to_wall_frame(pose, scan)
        if along is None:
            return None
        self.scans += 1
        for centre in self._clusters(along, depth):
            self._vote(centre)
        self.bay = self._resolve()
        return self.bay

    # ------------------------------------------------------------------
    def _to_wall_frame(self, pose, scan):
        """
        The scan as (along the wall, depth out from it), keeping only returns
        that could be a blade: in this section, and standing proud of the
        outer wall but not further out than a blade reaches.
        """
        import numpy as np

        scan = np.asarray(scan, dtype=float)
        bearings = np.arange(360.0)
        good = ~np.isnan(scan) & (scan > MIN_DETECT_RANGE_MM) & (scan < MAX_DETECT_RANGE_MM)
        if not np.any(good):
            return None, None

        # The beam starts at the lidar, not at the pose point (the rear axle).
        origin_x, origin_y = to_field(pose.x, pose.y, pose.heading, self.lidar_offset_mm)
        angles = np.radians(pose.heading + bearings[good])
        ranges = scan[good]
        x = origin_x + ranges * np.sin(angles)
        y = origin_y + ranges * np.cos(angles)

        wall_axis, sign = _SECTIONS[self.section]
        coord = (x, y)
        # Depth is measured out from the outer wall, so it is positive inside
        # the field whichever of the four walls this is.
        depth = self.map.outer - sign * coord[wall_axis]
        along = coord[1 - wall_axis]

        limit = self.map.inner      # the section runs +/- this along the wall
        keep = ((depth >= self.min_depth_mm)
                & (depth <= WALL_LENGTH_MM + self.min_depth_mm)
                & (np.abs(along) <= limit))
        if not np.any(keep):
            return None, None
        return along[keep], depth[keep]

    def _clusters(self, along, depth):
        """
        Blade candidates: runs of returns that sit close together ALONG the
        wall while reaching out from it. Yields each run's mean position.
        """
        import numpy as np

        order = np.argsort(along)
        along, depth = along[order], depth[order]
        splits = np.flatnonzero(np.diff(along) > CLUSTER_GAP_MM) + 1
        for group in np.split(np.arange(along.size), splits):
            if group.size < MIN_CLUSTER_POINTS:
                continue
            # A blade stands out from the wall; a smear along the wall at
            # constant depth is something else (a pillar, or the wall itself
            # seen through noise).
            if float(np.max(depth[group])) < self.min_depth_mm:
                continue
            yield float(np.mean(along[group]))

    def _vote(self, centre):
        index = int(round(centre / VOTE_BIN_MM))
        slot = self._votes.setdefault(index, [0, 0.0])
        slot[0] += 1
        slot[1] += centre
        self.blades += 1

    def _resolve(self):
        """
        Two well-supported blades the right distance apart, as a bay.

        Takes the best-supported pair rather than the first that fits, so a
        stray cluster that happens to sit a plausible distance from a real
        blade cannot beat the real pair.
        """
        candidates = [(count, total / count) for count, total in self._votes.values()
                      if count >= self.min_scans]
        if len(candidates) < 2:
            # Nothing can pair up - but one well-seen blade is still a bay.
            return self._from_one_blade()

        best = None
        for index, (count_a, position_a) in enumerate(candidates):
            for count_b, position_b in candidates[index + 1:]:
                gap = abs(position_a - position_b)
                if not self.min_gap_mm <= gap <= self.max_gap_mm:
                    continue
                support = min(count_a, count_b)
                if best is None or support > best[0]:
                    best = (support, (position_a + position_b) / 2.0, gap)
        if best is None:
            return self._from_one_blade()
        # Centre to centre spans the gap plus one blade's thickness.
        self.bay_mm = max(0.0, best[2] - WALL_THICKNESS_MM)
        return self.section, best[1]

    def _from_one_blade(self):
        """
        A bay placed from the NEAR blade alone, when only one is ever seen.

        Half the bay is often all there is to see. The far blade is edge-on
        from most of the approach, it is 10mm thick, and from the near side of
        it the near blade shadows the floor behind itself - so waiting for two
        clusters to agree can mean never parking at all.

        One blade is enough because the bay's WIDTH is not a measurement, it
        is a rule: the gap is fixed by the field, so the near blade's position
        plus half a bay along is the centre. The direction that "along" points
        is the one thing this cannot see for itself, which is why `travel_sign`
        is set from outside: the blade the robot meets first is the near one,
        so the bay is always AHEAD of it.

        Held to a higher bar than a pair - `single_scans` rather than
        `min_scans` - because a lone cluster has nothing corroborating it. A
        pillar standing near the wall looks exactly like one blade; what it
        cannot do is look like two blades the right distance apart.
        """
        if self.travel_sign is None:
            return None
        blades = [(count, total / count) for count, total in self._votes.values()
                  if count >= self.single_scans]
        if not blades:
            return None
        # The best-supported one: the nearest blade is the one seen most, and
        # a stray cluster seen twice should not outvote a wall seen twenty
        # times.
        _, position = max(blades, key=lambda blade: blade[0])
        self.bay_mm = NOMINAL_BAY_MM
        self.from_single_blade = True
        # Centre to centre spans the clear gap plus one blade's thickness, so
        # the bay centre is half of that beyond the blade's own centre.
        step = (NOMINAL_BAY_MM + WALL_THICKNESS_MM) / 2.0
        return self.section, position + self.travel_sign * step

    # ------------------------------------------------------------------
    def bay_point(self):
        """The middle of the bay's mouth, in field mm, or None."""
        if self.bay is None:
            return None
        x, y, _ = bay_pose(self.bay[0], self.bay[1], self.map, WALL_LENGTH_MM / 2.0)
        return x, y

    def status_line(self):
        if self.bay is None:
            return (f"bay: searching ({self.blades} blades over {self.scans} scans, "
                    f"{len(self._votes)} spots)")
        how = " (from ONE blade, width assumed)" if self.from_single_blade else ""
        return (f"bay: {self.bay[0]} at {self.bay[1] / 10:+.0f}cm, "
                f"{self.bay_mm / 10:.0f}cm wide{how}")


# ============================================================================
# Driving into the bay
# ============================================================================
# Where the robot's own body sits relative to the point that tracks the path.
# The pose point IS the rear axle (NavigationManager localizes the axle, with
# the lidar at its lidar_offset_mm ahead of it, and pursuit.rear_axle_offset_mm
# is 0), so these are measured from there.
# MEASURED on the robot, not nominal. Every clearance in this file is decided
# by these three, so a guess here is a guess about whether the park fits.
BODY_FRONT_MM = 170.0       # rear axle to front bumper
BODY_REAR_MM = 40.0         # rear axle to rear bumper - NOT yet measured
BODY_HALF_WIDTH_MM = 75.0   # half of a 150mm body


class BayFrame:
    """
    Converts a field pose into the only three numbers the manoeuvre cares
    about, so that none of the phase logic has to know which of the four walls
    it is working against or which way round the lap is being driven.

        s      along the wall, zero at the bay centre, POSITIVE the way the
               robot was travelling when it arrived
        d      depth: distance out from the outer wall, always positive
        theta  yaw, zero when square to the wall, POSITIVE when the nose has
               swung away from the wall

    With those, backing into a bay on the left going clockwise is the same
    arithmetic as backing into one on the right going anticlockwise, which is
    the entire reason this class exists.
    """

    def __init__(self, section, bay_centre_mm, field_map, travel_heading_deg,
                 wall_side):
        self.section = section
        self.map = field_map
        self.centre = float(bay_centre_mm)
        self.travel_heading = float(travel_heading_deg)
        self.wall_side = 1.0 if wall_side > 0 else -1.0
        self.wall_axis, self.sign = _SECTIONS[section]
        # Does travelling that way increase or decrease the along coordinate?
        reference = wall_heading_of(section)
        self.forward = 1.0 if abs(angle_difference(self.travel_heading,
                                                   reference)) <= 90.0 else -1.0

    def to_local(self, x, y, heading_deg):
        coord = (x, y)
        along = coord[1 - self.wall_axis]
        depth = self.map.outer - self.sign * coord[self.wall_axis]
        s = (along - self.centre) * self.forward
        # Nose away from the wall is positive whichever side the wall is on.
        theta = -self.wall_side * angle_difference(heading_deg, self.travel_heading)
        return s, depth, theta

    def corners_local(self, s, d, theta):
        """The chassis corners in (s, d), for the clearance guard."""
        radians = math.radians(theta)
        # Nose direction in the local frame: +s at theta 0, tilting to +d.
        nose = (math.cos(radians), math.sin(radians))
        # "Right" here means toward the wall, i.e. decreasing d at theta 0.
        side = (math.sin(radians), -math.cos(radians))
        return [(s + along * nose[0] + across * side[0],
                 d + along * nose[1] + across * side[1])
                for along, across in ((BODY_FRONT_MM, -BODY_HALF_WIDTH_MM),
                                      (BODY_FRONT_MM, BODY_HALF_WIDTH_MM),
                                      (-BODY_REAR_MM, BODY_HALF_WIDTH_MM),
                                      (-BODY_REAR_MM, -BODY_HALF_WIDTH_MM))]


class ParkingController:
    """
    The four-step parallel park, as a phase machine.

        APPROACH  come down off the racing line to `stage_depth_mm`, slowly
                  and with a short lookahead, holding parallel to the wall
        PULL      creep forward `pull_forward_mm` past the bay, steering the
                  yaw back to square as it goes
        ARC_IN    full lock toward the wall, reversing, until the yaw
                  reaches `turn_deg`
        STRAIGHT  wheels centred, reversing, until deep enough that the
                  closing arc will finish at `park_depth_mm`
        ARC_OUT   opposite full lock, still reversing, until square again
        SETTLE    nudge along the bay if the landing was off centre

    Every distance is a tunable, and the two that matter most are named for
    what you can actually see on the mat: `stage_depth_mm` is how far the
    robot sits off the outer wall before it starts, and
    `stage_along_offset_mm` is how far PAST the far wall its rear axle sits.

    GEOMETRY WARNING, at the measured lock this does not close. The two arcs
    move the robot sideways by 2R(1-cos t), which at R=197mm is 115mm at 45
    degrees, so the straight has to supply the rest of the depth - and the
    longer that straight is, the further back the manoeuvre ends. Worse, the
    closing arc sweeps the nose FORWARD: the front wall-side corner peaks
    around 33 degrees of yaw, and clearing the far wall there needs the axle
    159mm behind the bay centre, while the tail needs it no further back than
    110mm. Those do not overlap, at any staging depth, turn angle, park depth
    or end offset.

    It closes when either lever moves: a lock of about 65 degrees (R=77mm)
    clears by 8mm in a 300mm bay, or a bay of about 360mm clears at the
    measured lock. Both are outside this file. Everything below is written so
    that the day one of them changes, only numbers change.

    Never touches the motor: update() returns (steer, speed) and the task
    applies it, so `steer_command`/`speed` stay the single record of what the
    wheels were told and the odometry keeps working through the manoeuvre.
    """

    APPROACH, PULL, ARC_IN, STRAIGHT, ARC_OUT, SETTLE, DONE, ABORTED = (
        "approach", "pull", "arc_in", "straight", "arc_out", "settle",
        "done", "aborted")
    DRIVING = (PULL, ARC_IN, STRAIGHT, ARC_OUT, SETTLE)

    def __init__(self, frame, bay_mm=NOMINAL_BAY_MM,
                 turn_deg=45.0,
                 stage_depth_mm=350.0,
                 stage_along_offset_mm=None,
                 stage_at_wall="far",
                 park_depth_mm=105.0,
                 end_offset_mm=0.0,
                 straight_mm=None,
                 turn_radius_mm=196.6,
                 line_depth_mm=600.0,
                 approach_mm=900.0,
                 approach_speed=25,
                 approach_lookahead_mm=200.0,
                 approach_lean_deg=25.0,
                 pull_forward_mm=0.0,
                 arrival_window_mm=150.0,
                 straighten_deg=3.0,
                 straighten_extra_mm=200.0,
                 mm_per_s_at_full=400.0,
                 depth_gain=0.20,
                 approach_damping_deg=30.0,
                 heading_gain=2.0,
                 speed=25,
                 square_deg=3.0,
                 centre_tolerance_mm=15.0,
                 wall_guard_mm=10.0,
                 blade_guard_mm=6.0,
                 guard_enabled=True,
                 phase_timeout_s=25.0):
        self.frame = frame
        self.bay_mm = float(bay_mm)
        self.turn_deg = float(turn_deg)
        self.stage_depth_mm = float(stage_depth_mm)
        self.park_depth_mm = float(park_depth_mm)
        self.end_offset_mm = float(end_offset_mm)
        self.turn_radius_mm = float(turn_radius_mm)
        self.line_depth_mm = float(line_depth_mm)
        self.approach_mm = float(approach_mm)
        self.approach_speed = int(approach_speed)
        self.approach_lookahead_mm = float(approach_lookahead_mm)
        self.approach_lean_deg = float(approach_lean_deg)
        self.pull_forward_mm = max(0.0, float(pull_forward_mm or 0.0))
        self.arrival_window_mm = float(arrival_window_mm)
        self.straighten_deg = float(straighten_deg)
        self.straighten_extra_mm = max(0.0, float(straighten_extra_mm))
        self.mm_per_s_at_full = float(mm_per_s_at_full)
        self.depth_gain = float(depth_gain)
        self.approach_damping_deg = float(approach_damping_deg)
        self.heading_gain = float(heading_gain)
        self.speed = int(speed)
        self.square_deg = float(square_deg)
        self.centre_tolerance_mm = float(centre_tolerance_mm)
        self.wall_guard_mm = float(wall_guard_mm)
        self.blade_guard_mm = float(blade_guard_mm)
        self.guard_enabled = bool(guard_enabled)
        self.phase_timeout_s = float(phase_timeout_s)

        self.phase = self.APPROACH
        self.reason = None
        self.s = self.d = self.theta = 0.0
        self._elapsed = 0.0
        self._pulled_mm = 0.0
        self._declined = False
        self._straight_done_mm = 0.0

        self.stage_at_wall = ("near" if str(stage_at_wall).lower() == "near"
                              else "far")
        self.straight_mm, auto_offset = self.plan()
        # Where the arcs have to START for the body to land centred - solved,
        # and always measured from the far wall whichever end the robot lines
        # up against. `suggested_pull_mm` is the gap between that and the
        # staging point, which is what PULL exists to cover.
        self._arc_start_mm = self.bay_mm / 2.0 + auto_offset
        if stage_along_offset_mm is not None:
            self.stage_along_offset_mm = float(stage_along_offset_mm)
        elif self.stage_at_wall == "near":
            # Level with the near wall's inner face. The solved offset is a
            # FAR-wall number and means nothing at this end.
            self.stage_along_offset_mm = 0.0
        else:
            self.stage_along_offset_mm = auto_offset
        if straight_mm is not None:
            self.straight_mm = float(straight_mm)

    # ------------------------------------------------------------------
    # PLANNING
    # ------------------------------------------------------------------
    def plan(self):
        """
        The straight's length, and where the rear axle has to start.

        Solved rather than tuned, from the two things the manoeuvre has to
        achieve: end at `park_depth_mm`, and end with the body at
        `end_offset_mm` along the bay.

            depth   stage_depth - 2R(1-cos t) - L sin t = park_depth
            along   stage_s     - 2R sin t     - L cos t = end_offset - 80

        The offset is returned relative to the FAR wall's inner face, because
        that is the thing you can see from outside the robot: zero means the
        rear axle is level with it, which is where a driver would start.

        I/O:
            return: (straight_mm, stage_along_offset_mm)
        """
        turn = math.radians(self.turn_deg)
        radius = self.turn_radius_mm
        lift = 2.0 * radius * (1.0 - math.cos(turn))
        straight = (self.stage_depth_mm - self.park_depth_mm - lift) / max(1e-3, math.sin(turn))
        axle_end = self.end_offset_mm - (BODY_FRONT_MM - BODY_REAR_MM) / 2.0
        stage_s = axle_end + 2.0 * radius * math.sin(turn) + straight * math.cos(turn)
        return straight, stage_s - self.bay_mm / 2.0

    @property
    def stage_s_mm(self):
        """
        Where the rear axle lines up, in bay coordinates.

        Two ends to choose from, and the difference is which wall the robot
        stops level with:

        "far"  - past BOTH blades, at the solved offset, and the arcs start
                 straight away. One number, solved, nothing to see from
                 outside until it is already reversing.
        "near" - level with the inner face of the FIRST blade it reaches, then
                 PULL forward to the arc start. The stopping point is then a
                 wall the robot has just driven past rather than a computed
                 point in open floor, and the distance from it to the arc
                 start is one number you can lay a tape along.
        """
        end = -1.0 if self.stage_at_wall == "near" else 1.0
        return end * self.bay_mm / 2.0 + self.stage_along_offset_mm

    @property
    def suggested_pull_mm(self):
        """
        The forward run that carries the robot from where it lines up to where
        the arcs have to start. What `pull_forward_mm` wants to be, before any
        mat correction for a turning radius that is not what the config says.
        """
        return max(0.0, self._arc_start_mm - self.stage_s_mm)

    def summary(self):
        wall = "near" if self.stage_at_wall == "near" else "far"
        pull = (f"pull {self.pull_forward_mm:.0f}mm  " if self.pull_forward_mm else "")
        note = f"  square to {self.straighten_deg:.0f}deg before reversing"
        if self.stage_at_wall == "near" and abs(
                self.pull_forward_mm - self.suggested_pull_mm) > 20.0:
            note = (f"  [pull_forward_mm wants about "
                    f"{self.suggested_pull_mm:.0f} to land centred]")
        return (f"line up at the {wall} wall, axle "
                f"{self.stage_along_offset_mm:+.0f}mm past its inner face, "
                f"{self.stage_depth_mm:.0f}mm out  {pull}"
                f"turn {self.turn_deg:.0f}deg  R {self.turn_radius_mm:.0f}mm  "
                f"straight {self.straight_mm:.0f}mm  park {self.park_depth_mm:.0f}mm{note}")

    # ------------------------------------------------------------------
    # THE LOOP
    # ------------------------------------------------------------------
    def update(self, pose, dt, max_steer=40.0):
        """
        One tick of the manoeuvre.

        I/O:
            pose: current Pose, in field coordinates
            return: (steer_command, speed), or None while the path follower
                    should stay in charge, or when the manoeuvre is over
        """
        if self.phase in (self.DONE, self.ABORTED):
            return None
        self.s, self.d, self.theta = self.frame.to_local(pose.x, pose.y, pose.heading)
        self._elapsed += dt
        max_steer = float(max_steer)

        if self.phase == self.APPROACH:
            return self._approach(dt, max_steer)

        if self._elapsed > self.phase_timeout_s:
            return self._abort(f"{self.phase} timed out after {self._elapsed:.1f}s")
        if self.guard_enabled:
            breach = self._breach_mm()
            if breach is not None:
                return self._abort(breach)

        return {self.PULL: self._pull, self.ARC_IN: self._arc_in,
                self.STRAIGHT: self._straight, self.ARC_OUT: self._arc_out,
                self.SETTLE: self._settle}[self.phase](dt, max_steer)

    # ------------------------------------------------------------------
    # STEP 0 and 1 - get to the staging pose
    # ------------------------------------------------------------------
    def _approach(self, dt, max_steer):
        """
        Come down off the racing line to the staging pose, and stop there.

        Two things happen at once, and both are step 0: the speed and the
        lookahead are cut (see path_caps, which the task applies to the path
        follower), and the line is left for a shallow slide in to
        `stage_depth_mm`. The slide is a lean, not a turn - `approach_lean_deg`
        caps how far off parallel it will go - so the robot arrives square,
        which is what the first arc is measured from.
        """
        remaining = self.stage_s_mm - self.s
        if remaining > self.approach_mm:
            self._declined = False           # a fresh run at it next time round
            return None                      # the racing line still has it
        if remaining <= 0.0:
            # ARRIVED, or MISSED? The difference matters more than it looks,
            # because what follows is irreversible: the next thing this does
            # is reverse on full lock, and it does that from wherever the
            # robot is standing when this returns.
            #
            # `remaining <= 0` alone does not mean "arrived". It also reads
            # true when the manoeuvre is armed LATE - the robot is already
            # past the staging point when parking becomes allowed, which
            # parking.start_early_mm makes easy to do - and the robot is then
            # somewhere down the wall at racing-line depth, nowhere near the
            # bay. Committing there parks in open floor, short of the bay by
            # however late it armed.
            #
            # So both have to hold: past the staging point but only just, and
            # already down at staging depth. Otherwise this hands the wheels
            # back and lets the lap carry the robot round for a proper run at
            # it - a lap is cheap next to a park in the wrong place.
            missed = remaining < -self.arrival_window_mm
            off_depth = abs(self.d - self.stage_depth_mm) > self.arrival_window_mm
            if missed or off_depth:
                if not self._declined:
                    self._declined = True
                    why = []
                    if missed:
                        why.append(f"{-remaining:.0f}mm past the staging point")
                    if off_depth:
                        why.append(f"{self.d:.0f}mm off the wall, wanted "
                                   f"{self.stage_depth_mm:.0f}")
                    print(f"Parking: not committing - {' and '.join(why)}. "
                          f"Going round for another run at it.")
                return None
            self._enter(self.PULL)
            return (0.0, 0)

        # Lean toward the staging depth, then hold that lean.
        #
        # DAMPED, because the obvious version overshoots badly. Depth error
        # only falls BECAUSE the robot is yawed inward, so a lean proportional
        # to the error alone is still pointing at the wall at the moment the
        # error reaches zero, and the robot sails through the staging depth -
        # measured at 140mm past a 400mm target, which puts the body's
        # wall-side edge inside the 200mm the blades stick out. The first
        # thing it then meets is a blade.
        #
        # The damping term is the closing rate: depth falls by sin(theta) per
        # millimetre travelled, so subtracting it backs the lean off as the
        # robot is already on its way in, and reverses it near the end. That
        # is what makes the approach ARRIVE at a depth rather than pass
        # through it.
        depth_error = self.d - self.stage_depth_mm       # + = still too far out
        closing = math.sin(math.radians(self.theta))     # - = closing on the wall
        wanted = clamp(-self.depth_gain * depth_error
                       - self.approach_damping_deg * closing,
                       -self.approach_lean_deg, self.approach_lean_deg)
        steer = clamp(self.heading_gain * (wanted - self.theta), -max_steer, max_steer)
        # Forward travel, so the wheel goes the opposite way to the yaw it is
        # asking for - see _command_for, which this is the proportional twin of.
        return (-self.frame.wall_side * steer, self.approach_speed)

    def path_caps(self):
        """
        What the path follower should be limited to right now, as
        (speed, lookahead_mm). Either may be None.

        This is step 0. A long lookahead is what makes the robot cut a corner,
        and cutting the corner here means arriving at the staging point off
        line and off square, which the rest of the manoeuvre has no way to
        correct - every later step is an open arc measured from this pose.
        """
        if self.phase != self.APPROACH:
            return (None, None)
        if self.s < self.stage_s_mm - self.approach_mm:
            return (None, None)
        return (self.approach_speed, self.approach_lookahead_mm)

    # ------------------------------------------------------------------
    # STEPS 2, 3, 4
    # ------------------------------------------------------------------
    def _pull(self, dt, max_steer):
        """
        Forward past the bay, straightening up, before any reversing starts.

        What a driver does: pull level with the car in front, square up, THEN
        reverse. Two jobs, and both matter:

        THE DISTANCE. The staging point alone is a solved number - it assumes
        the two arcs carry the robot back by exactly 2R sin(t) + L cos(t) -
        and R is the least trustworthy figure in the manoeuvre, because it
        comes from pursuit.max_road_wheel_deg rather than from anything
        measured here. If the robot really turns less than that says, every
        reverse leg travels further back along the wall than planned and the
        park lands short, on the NEAR wall. This is the mat-measurable
        correction: drive on by however much it lands short. Arithmetically
        the same as raising `stage_along_offset_mm`, and deliberately separate
        because it is a thing you can watch happen and measure with a tape.

        THE HEADING. Every step after this is an OPEN ARC measured from this
        pose - nothing downstream reads the yaw again until ARC_IN is counting
        up to turn_deg from it. So a robot that arrives leaning takes that
        lean through the whole manoeuvre: the first arc ends at the wrong
        angle, the straight then stops at the wrong depth, and the closing arc
        squares up to a wall it is no longer parallel to. The approach gets
        the robot to the right DEPTH by leaning (see approach_lean_deg), which
        means it is leaning by construction when it arrives. Straightening it
        is therefore not a refinement - it is undoing the approach's own
        method before the open-loop part starts.

        A car cannot yaw standing still, so the straightening rides on the
        forward run rather than costing a phase of its own. The phase ends
        when the distance is done AND the yaw is inside `straighten_deg`; if
        the yaw will not come in, it gives up after `straighten_extra_mm` more
        rather than creeping until the phase times out.
        """
        if self.pull_forward_mm <= 0.0 and abs(self.theta) <= self.straighten_deg:
            self._enter(self.ARC_IN)
            return (0.0, 0)

        far_enough = self._pulled_mm >= self.pull_forward_mm
        square = abs(self.theta) <= self.straighten_deg
        if far_enough and square:
            self._enter(self.ARC_IN)
            return (0.0, 0)
        if self._pulled_mm >= self.pull_forward_mm + self.straighten_extra_mm:
            print(f"Parking: still {self.theta:+.1f}deg off square after "
                  f"{self._pulled_mm:.0f}mm - backing in anyway")
            self._enter(self.ARC_IN)
            return (0.0, 0)

        self._pulled_mm += abs(self.approach_speed) / 100.0 * self.mm_per_s_at_full * dt
        # Steer the yaw back to zero. Forward travel, so the wheel goes the
        # opposite way to the yaw being asked for - the same relation
        # _approach uses, with a wanted lean of nothing.
        steer = clamp(-self.heading_gain * self.theta, -max_steer, max_steer)
        return (-self.frame.wall_side * steer, self.approach_speed)

    def _arc_in(self, dt, max_steer):
        """Step 2: full lock toward the wall, reversing, until `turn_deg`."""
        if self.theta >= self.turn_deg:
            self._enter(self.STRAIGHT)
            return (0.0, 0)
        return (self._command_for(-1.0, 1.0 / self.turn_radius_mm, max_steer),
                -self.speed)

    def _straight(self, dt, max_steer):
        """
        Step 3: wheels centred, reversing, until the closing arc will finish
        at parking depth.

        Ended on DEPTH rather than on distance travelled. They are the same
        thing if the first arc came out exactly at `turn_deg`, and they are
        not if it did not - and the arc is the part most likely to be off,
        because it is the one the servo's own lag distorts.
        """
        stop_at = self.park_depth_mm + self.turn_radius_mm * (
            1.0 - math.cos(math.radians(max(1.0, self.theta))))
        # Off the commanded speed, so it tracks whatever the gearing is -
        # this used to be a hardcoded 700mm/s and quietly became a 75%
        # overestimate the day the gearbox changed.
        self._straight_done_mm += abs(self.speed) / 100.0 * self.mm_per_s_at_full * dt
        if self.d <= stop_at or self._straight_done_mm > 2.0 * self.straight_mm + 100.0:
            self._enter(self.ARC_OUT)
            return (0.0, 0)
        return (0.0, -self.speed)

    def _arc_out(self, dt, max_steer):
        """Step 4: opposite lock, still reversing, until square."""
        if self.theta <= self.square_deg:
            self._enter(self.SETTLE)
            return (0.0, 0)
        return (self._command_for(-1.0, -1.0 / self.turn_radius_mm, max_steer),
                -self.speed)

    def _settle(self, dt, max_steer):
        """A straight nudge along the bay, once it is square."""
        error = self.s - self._centring_s_mm(self.theta)
        if abs(error) <= self.centre_tolerance_mm:
            self.phase = self.DONE
            return (0.0, 0)
        return (0.0, -self.speed if error > 0.0 else self.speed)

    # ------------------------------------------------------------------
    # HOUSEKEEPING
    # ------------------------------------------------------------------
    def _enter(self, phase):
        self.phase = phase
        self._elapsed = 0.0

    def _abort(self, reason):
        self.phase = self.ABORTED
        self.reason = reason
        print(f"Parking aborted: {reason}")
        return (0.0, 0)

    def _command_for(self, direction, kappa, max_steer):
        """
        The steering command that produces `kappa` - yaw per millimetre
        TRAVELLED - while moving in `direction`.

        Reversing flips which way the wheels must point, and so does having
        the wall on the other side, which is why this is derived once rather
        than written out per phase.
        """
        if not kappa:
            return 0.0
        return (-math.copysign(1.0, kappa) * self.frame.wall_side
                * direction * max_steer)

    @staticmethod
    def _centring_s_mm(theta_deg):
        """The axle position that centres the body, at a yaw of theta."""
        return (-(BODY_FRONT_MM - BODY_REAR_MM) / 2.0
                * math.cos(math.radians(theta_deg)))

    def _breach_mm(self):
        return self._breach_at(self.s, self.d, self.theta)

    def _breach_at(self, s, d, theta):
        """
        What, if anything, the chassis would go through at (s, d, theta).

        A separating-axis test against each wall's real 10mm-thick box, not a
        corner-in-box test. The difference matters here: the way this
        manoeuvre touches a wall is the rear EDGE grazing the far wall's top
        outer corner as the tail swings in, and every corner of the robot is
        somewhere else entirely when that happens.
        """
        half = self.bay_mm / 2.0
        body = list(self.frame.corners_local(s, d, theta))
        if min(c[1] for c in body) < self.wall_guard_mm:
            return (f"corner {self.wall_guard_mm - min(c[1] for c in body):.0f}mm "
                    f"inside the outer wall")
        guard = self.blade_guard_mm
        for sign in (-1.0, 1.0):
            near = sign * half
            far = sign * (half + WALL_THICKNESS_MM)
            box = (min(near, far) - guard, max(near, far) + guard,
                   -guard, WALL_LENGTH_MM + guard)
            if self._overlaps(body, box):
                return f"chassis is through the bay wall at s={near:+.0f}"
        return None

    @staticmethod
    def _overlaps(body, box):
        """Separating-axis test: rotated rectangle against an axis-aligned box."""
        corners = [(box[0], box[2]), (box[1], box[2]),
                   (box[1], box[3]), (box[0], box[3])]
        axes = [(1.0, 0.0), (0.0, 1.0)]
        for i in range(2):
            ex = body[i + 1][0] - body[i][0]
            ey = body[i + 1][1] - body[i][1]
            length = math.hypot(ex, ey)
            if length:
                axes.append((-ey / length, ex / length))
        for ax in axes:
            a = [p[0] * ax[0] + p[1] * ax[1] for p in body]
            b = [p[0] * ax[0] + p[1] * ax[1] for p in corners]
            if max(a) <= min(b) or min(a) >= max(b):
                return False
        return True

    # ------------------------------------------------------------------
    @property
    def active(self):
        """True when the manoeuvre, rather than the path, owns the wheels."""
        return self.phase in self.DRIVING

    @property
    def finished(self):
        return self.phase in (self.DONE, self.ABORTED)

    def status_line(self):
        return (f"park {self.phase:8} s={self.s:+7.1f} d={self.d:6.1f} "
                f"yaw={self.theta:+6.1f}")


class UnparkController:
    """
    Getting OUT of the bay, at the START of a run.

    Deliberately not the park run backwards. The park has to land the body
    between two walls it cannot touch, so every step of it is measured off the
    filter's pose; leaving only has to end up somewhere on the track pointing
    roughly the right way, and the pose at the moment the run starts is the
    LEAST trustworthy one of the whole round - the filter has just converged,
    from inside a slot whose two walls look like nothing else on the mat. So
    this is open loop, off the lidar and the odometry, and short:

        LOOK      stand still and ask the lidar which side of the robot has
                  room in it, because that is the way out
        SET_BACK  stand still and swing the steering to the OPPOSITE lock
        REVERSE   back up `reverse_mm` on that opposite lock, to buy room in
                  front and to aim the nose at the way out
        SET_OUT   stand still again and swing the steering to the exit lock,
                  so the wheels are already there when the robot starts
                  moving rather than arriving a metre into the move
        FORWARD   drive forward at that lock until far enough out, then hand
                  the wheels to the path follower

    WHY REVERSE FIRST. The turn out is an arc, and an arc needs length before
    it has moved the robot sideways at all - 2R(1-cos t), which at the
    measured lock is 15mm in the first 10 degrees of yaw. Started hard against
    the front of the bay that length is not there, and the nose sweeps into
    the far wall while the body is still barely out of the slot. Backing up
    first spends that length where there is nothing to hit.

    WHY THE OPPOSITE LOCK WHILE REVERSING. Yaw rate is (v/L)tan(d), so
    reversing NEGATES it: back up on left lock and the body turns the same way
    as driving forward on right lock. Both legs therefore rotate the robot the
    SAME way, and the reverse arrives at the mouth of the bay with the nose
    already aimed at the open side instead of having to develop all of that
    yaw from inside the slot, where there is no room for it.

    The price is at the other end of the body: the tail swings toward the far
    wall while the nose comes round, so `reverse_mm` stays short, and
    `reverse_steer_command` defaults to 0 - straight back, no counter-steer -
    for a bay too tight to give the tail anywhere to go.

    THREE NUMBERS ARE NOT FILLED IN. `reverse_mm` (how far back), then
    `steer_command` (how hard to turn out) and `forward_mm` (how far to drive
    on that lock) are the whole manoeuvre, and they are the ones you can only
    get off the mat with a tape measure - they depend on where in the bay the
    robot was placed and how much lock the servo really has. Until ALL THREE
    are set - unpark.reverse_mm, unpark.steer_command and unpark.forward_mm -
    this refuses to drive at all and says so, rather than guessing and driving
    into a bay wall. Zero is a legal `reverse_mm`, and means "no reverse".

    `steer_command` and `reverse_steer_command` are MAGNITUDES. Which way
    either points is not a tunable: the exit lock goes toward whichever side
    the lidar found open, the reverse lock goes the other way, and both signs
    are applied here.

    Never touches the motor: update() returns (steer, speed) and the task
    applies it, the same contract as ParkingController, so the odometry and
    the status line keep working through the manoeuvre.
    """

    LOOK, SET_BACK, REVERSE, SET_OUT, FORWARD, DONE, ABORTED = (
        "look", "set_back", "reverse", "set_out", "forward", "done", "aborted")
    # Every phase before the end owns the wheels - the standing-still ones
    # included, because standing still is a thing the robot has to be TOLD to
    # do.
    DRIVING = (LOOK, SET_BACK, REVERSE, SET_OUT, FORWARD)

    # What an empty sector is worth when the two sides are compared. Nothing
    # coming back is not missing data here, it is the most open answer there
    # is: it means the nearest thing that way is past the lidar's useful
    # range, which for the 3m field means there is nothing that way at all.
    NO_RETURN_MM = MAX_DETECT_RANGE_MM

    def __init__(self, lidar=None,
                 reverse_mm=None,
                 reverse_steer_command=0.0,
                 steer_command=None,
                 forward_mm=None,
                 side=None,
                 speed=25,
                 reverse_speed=20,
                 look_s=0.5,
                 servo_settle_s=0.4,
                 side_bearing_deg=90.0,
                 side_sector_deg=45.0,
                 side_margin_mm=100.0,
                 in_bay_mm=250.0,
                 default_side=+1,
                 mm_per_s_at_full=400.0,
                 timeout_s=15.0):
        self.lidar = lidar
        self.reverse_mm = None if reverse_mm is None else abs(float(reverse_mm))
        self.reverse_steer_command = abs(float(reverse_steer_command or 0.0))
        self.steer_command = None if steer_command is None else abs(float(steer_command))
        self.forward_mm = None if forward_mm is None else float(forward_mm)
        self.speed = int(speed)
        self.reverse_speed = abs(int(reverse_speed))
        self.look_s = float(look_s)
        self.servo_settle_s = float(servo_settle_s)
        self.side_bearing_deg = float(side_bearing_deg)
        self.side_sector_deg = float(side_sector_deg)
        self.side_margin_mm = float(side_margin_mm)
        # How close the nearer wall has to be for this to be a bay at all -
        # see _decide_side. 250mm is comfortably above the ~95mm a bay leaves
        # and well below anything on the open track.
        self.in_bay_mm = float(in_bay_mm)
        self.default_side = 1 if int(default_side) >= 0 else -1
        self.mm_per_s_at_full = float(mm_per_s_at_full)
        self.timeout_s = float(timeout_s)

        # +1 exits to the right, -1 to the left. Set by hand to skip the look,
        # otherwise decided in LOOK.
        self.side = None if side is None else (1 if int(side) >= 0 else -1)
        # Whether that side came from two clearly different readings, or from
        # default_side breaking a tie. Only a confident side is worth deciding
        # the LAP direction on - see wall_side.
        self.side_confident = False
        # The signed commands, once the side is known: out toward the open
        # side, back the other way.
        self.steer = 0.0
        self.reverse_steer = 0.0
        self.left_mm = self.right_mm = float("nan")
        # Whether the look actually found a bay around the robot. False on a
        # track start, and then the reading says nothing about lap direction.
        self.in_bay = False
        self.reversed_mm = 0.0
        self.driven_mm = 0.0
        self.reason = None

        self._elapsed = 0.0
        self._looks = 0
        self._left_sum = self._right_sum = 0.0

        self.phase = self.LOOK
        missing = [name for name, value in (("unpark.reverse_mm", self.reverse_mm),
                                            ("unpark.steer_command", self.steer_command),
                                            ("unpark.forward_mm", self.forward_mm))
                   if value is None]
        if missing:
            named = (missing[0] if len(missing) == 1
                     else f"{', '.join(missing[:-1])} and {missing[-1]}")
            self._abort(f"{named} not set - "
                        f"measure them on the mat and put them in the config")

    # ------------------------------------------------------------------
    def summary(self):
        steer = "unset" if self.steer_command is None else f"{self.steer_command:.0f}"
        forward = "unset" if self.forward_mm is None else f"{self.forward_mm:.0f}mm"
        back = "unset" if self.reverse_mm is None else f"{self.reverse_mm:.0f}mm"
        counter = (f"counter-lock {self.reverse_steer_command:.0f}"
                   if self.reverse_steer_command else "wheels centred")
        side = "from the lidar" if self.side is None else self.side_name(self.side)
        return (f"exit {side}: back {back} at speed {self.reverse_speed} "
                f"({counter}), lock {steer}, forward {forward} at speed {self.speed}")

    @staticmethod
    def side_name(side):
        return "right" if side >= 0 else "left"

    # ------------------------------------------------------------------
    # THE LOOP
    # ------------------------------------------------------------------
    def update(self, pose, dt, max_steer=40.0):
        """
        One tick of the exit.

        I/O:
            pose: current Pose - unused, this is open loop; taken so that the
                  task can drive either manoeuvre through the same call
            return: (steer_command, speed), or None once it is over and the
                    path follower should take the wheels
        """
        if self.finished:
            return None
        self._elapsed += dt
        if self._elapsed > self.timeout_s:
            return self._abort(f"{self.phase} timed out after {self._elapsed:.1f}s")

        if self.phase == self.LOOK:
            return self._look(dt, max_steer)
        if self.phase == self.SET_BACK:
            return self._set_back(dt)
        if self.phase == self.REVERSE:
            return self._reverse(dt)
        if self.phase == self.SET_OUT:
            return self._set_out(dt)
        return self._forward(dt)

    # ------------------------------------------------------------------
    def _look(self, dt, max_steer):
        """
        Stand still and find the open side.

        Averaged over a few ticks rather than read once: a single scan can put
        a nan in either sector, and picking the way out off one unlucky frame
        is how the robot drives into the wall it was parked against.
        """
        if self.side is None:
            self._sample()
            # A tick with no lidar at all never accumulates a sample, so the
            # sample count cannot be what holds this open, or a run without
            # one sits in the bay until the timeout.
            if self._elapsed < self.look_s or (self.lidar is not None
                                               and self._looks == 0):
                return (0.0, 0)
            self.side = self._decide_side()
            if not self.in_bay and self.lidar is not None:
                # Nothing to get out of. Reversing and swinging out of a bay
                # that is not there just puts the robot somewhere the racing
                # line then has to recover from.
                return self._abort("not started in a bay - nothing to unpark from")
        self.steer = clamp(self.side * self.steer_command, -max_steer, max_steer)
        # The other way, so that reversing rotates the robot the SAME way the
        # forward leg will - see the class docstring.
        self.reverse_steer = (clamp(-self.side * self.reverse_steer_command,
                                    -max_steer, max_steer)
                              if self.reverse_steer_command else 0.0)
        print(f"Unparking to the {self.side_name(self.side)}: "
              f"left {self._range_text(self.left_mm)}, "
              f"right {self._range_text(self.right_mm)}")
        self._enter(self.SET_BACK)
        return (self.reverse_steer, 0)

    def _set_back(self, dt):
        """Hold still while the servo reaches the counter-lock."""
        return self._hold(self.reverse_steer, self.REVERSE)

    def _set_out(self, dt):
        """Hold still while the servo reaches the exit lock."""
        return self._hold(self.steer, self.FORWARD)

    def _hold(self, steer, then):
        """
        Stand still with the wheels already turned, then move on.

        The steering is a servo with a real travel time, and each leg starts
        from wherever the last one left the wheels. Asking for lock and drive
        on the same tick spends the first part of the move going straight,
        which is exactly the part that has a bay wall in it. Skipped outright
        when the wheels are already where they need to be, so a manoeuvre with
        no counter-steer does not stand around waiting for a servo that has
        nowhere to travel.
        """
        if steer == 0.0 or self._elapsed >= self.servo_settle_s:
            self._enter(then)
        return (steer, 0)

    def _reverse(self, dt):
        """
        Back up to buy the room the turn out needs, on the opposite lock.

        Measured by odometry like the forward leg, and slower, because what is
        behind the robot is the wall it was parked against and the
        commanded-speed estimate is all there is to stop against it.
        """
        if self.reverse_mm <= 0.0 or self.reversed_mm >= self.reverse_mm:
            self._enter(self.SET_OUT)
            return (self.reverse_steer, 0)
        self.reversed_mm += self._step_mm(self.reverse_speed, dt)
        return (self.reverse_steer, -self.reverse_speed)

    def _forward(self, dt):
        """Drive out on that lock, measuring by odometry, then stop steering."""
        self.driven_mm += self._step_mm(self.speed, dt)
        if self.driven_mm >= self.forward_mm:
            self.phase = self.DONE
            return (0.0, 0)
        return (self.steer, self.speed)

    def _step_mm(self, speed, dt):
        """How far a tick at this commanded speed moves the robot."""
        return abs(speed) / 100.0 * self.mm_per_s_at_full * dt

    # ------------------------------------------------------------------
    # WHICH WAY IS OUT
    # ------------------------------------------------------------------
    def _sample(self):
        """Folds this tick's two side ranges into the running means."""
        if self.lidar is None:
            return
        left = self._sector_min(-self.side_bearing_deg)
        right = self._sector_min(+self.side_bearing_deg)
        self._left_sum += left
        self._right_sum += right
        self._looks += 1
        self.left_mm = self._left_sum / self._looks
        self.right_mm = self._right_sum / self._looks

    def _sector_min(self, bearing_deg):
        """
        Closest return in a sector centred on `bearing_deg`, with an empty
        sector reading as wide open - see NO_RETURN_MM.
        """
        half = self.side_sector_deg / 2.0
        distance, _ = self.lidar.get_min_distance(bearing_deg - half,
                                                  bearing_deg + half)
        if distance is None or math.isnan(distance):
            return self.NO_RETURN_MM
        return min(float(distance), self.NO_RETURN_MM)

    def _decide_side(self):
        """
        The open side, or `default_side` when the two are too close to call.

        A margin, not a plain comparison: parked square in a 300mm bay both
        sides read the same wall a few millimetres apart, and a difference
        that small is noise, not information about where the track is.
        """
        if self._looks == 0:
            print("WARNING: no lidar to unpark by - "
                  f"exiting {self.side_name(self.default_side)} on faith")
            return self.default_side
        if abs(self.right_mm - self.left_mm) < self.side_margin_mm:
            print(f"Unpark: both sides within {self.side_margin_mm:.0f}mm of each "
                  f"other, going {self.side_name(self.default_side)} by default")
            return self.default_side
        # IN A BAY, OR JUST STANDING ON THE TRACK? The comparison below is only
        # ever meaningful between two bay walls. Out on the mat the two sectors
        # read hundreds of millimetres either way and they are never equal, so
        # the margin test above passes on any start at all and the wider side
        # gets reported as "the track" with full confidence - which
        # FinalTask._lap_direction then trusts over the racing line and runs
        # the whole round, and the park, backwards. A bay pins BOTH sides
        # close: 340mm of slot around a 150mm body is under 100mm a side.
        self.in_bay = min(self.left_mm, self.right_mm) <= self.in_bay_mm
        self.side_confident = self.in_bay
        if not self.in_bay:
            print(f"Unpark: nearest wall is "
                  f"{min(self.left_mm, self.right_mm):.0f}mm away, further than the "
                  f"{self.in_bay_mm:.0f}mm a bay puts it - this is not a bay start, "
                  f"so the lap direction is left to the racing line")
        return 1 if self.right_mm > self.left_mm else -1

    @property
    def wall_side(self):
        """
        Which side of the robot the bay's outer wall is on: +1 right, -1 left,
        None when the look could not tell.

        The bay is a slot in the OUTER wall, so the wall is whichever side the
        track is not - the exact opposite of the side being left by. Worth
        having as its own name because it answers a question the manoeuvre
        itself does not care about: which way round the lap runs. The wall is
        measured here in the robot's own frame, from two lidar sectors, with
        no pose and no map in the chain.
        """
        if self.side is None or not self.side_confident:
            return None
        return -self.side

    # ------------------------------------------------------------------
    # HOUSEKEEPING
    # ------------------------------------------------------------------
    def _enter(self, phase):
        self.phase = phase
        self._elapsed = 0.0

    def _abort(self, reason):
        self.phase = self.ABORTED
        self.reason = reason
        print(f"Unparking skipped: {reason}")
        return (0.0, 0)

    def _range_text(self, value):
        if math.isnan(value):
            return "unknown"
        if value >= self.NO_RETURN_MM:
            return "clear"
        return f"{value:.0f}mm"

    @property
    def active(self):
        """True when the manoeuvre, rather than the path, owns the wheels."""
        return self.phase in self.DRIVING

    @property
    def finished(self):
        return self.phase in (self.DONE, self.ABORTED)

    def status_line(self):
        return (f"unpark {self.phase:7} "
                f"side={'?' if self.side is None else self.side_name(self.side)} "
                f"L={self._range_text(self.left_mm)} R={self._range_text(self.right_mm)} "
                f"back={self.reversed_mm:.0f}mm out={self.driven_mm:.0f}mm")


class ParkingSequence:
    """
    Parking as a script the lidar starts, not a trajectory the map plans.

    WHY THIS REPLACED THE FOUR-STEP. The old ParkingController solved the park
    as geometry: find the bay on the map, build a frame on it, and drive four
    arcs measured from the robot's pose. Every one of those steps leans on the
    localizer, and the localizer is at its worst exactly here - a robot beside
    a bay sees a scan unlike anywhere else on the field, and a pose 900mm out
    reports a section, a wall and a staging point with complete confidence. It
    also could not close: at the measured 39 degrees of lock, in a 340mm bay,
    360 of 360 parameter combinations ended with the body through a bay wall.

    This does none of that. It drives beside the wall on a lidar range, waits
    for the wall to step closer - which is the bay wall and nothing else - and
    then runs a fixed sequence of servo angles and distances. The whole thing
    is five numbers you measure with a tape on the mat, and not one of them
    comes from the pose.

        FOLLOW    drive alongside the outer wall, holding `wall_distance_mm`
                  AND holding the body parallel to it, both on the side lidar
        TRIGGER   the side sector drops below `trigger_below_mm` - a bay wall
                  stands 200mm proud of the outer wall, so nothing else does
                  this - and the camera agrees there is pink over there
        CREEP     roll on until the NARROW beam drops too, which is the axle
                  coming level with the first blade, then `turn_after_mm` more
                  - holding the wall's heading on the compass the whole way,
                  because the side lidar is looking at a bay wall by now and
                  cannot say which way the robot is pointing any more
        MEASURE   carry on to the SECOND bay wall, so the bay's real width -
                  and so its real middle - is measured rather than assumed
        SETTLE    keep going a little past it, where the side lidar is back on
                  clean wall, and square up on it - everything after this is
                  open loop, so this is the pose the whole manoeuvre is
                  measured from. The road spent doing it is added to the
                  reverse, so it does not move where the robot ends up
        BACK_CENTRE reverse to the point the turn has to start from, which is
                  the middle of the bay less the radius the turn swings the
                  axle through
        SET_TURN  stand still while the servo winds on to full lock, so the
                  turn is the arc it was planned as rather than a straight
                  first half-metre followed by a tighter one
        TURN_IN   turn `turn_in_deg` toward the wall, ON THE COMPASS, so the
                  robot ends between the two blades facing the outer wall
        NOSE_IN   drive in until the NOSE is `nose_stop_mm` off the outer
                  wall - the beam sits `lidar_ahead_mm` forward of the axle,
                  so what it has to read is that much less - squaring up to
                  the wall on the compass as it goes, which takes out whatever
                  the turn overshot by, and stop

    HEAD IN, SQUARE TO THE WALL. Not a parallel park: at 39 degrees of lock a
    210mm body cannot be threaded lengthwise into a 340mm slot on an arc - the
    best of 630,000 swept combinations still swept 50mm through a blade on the
    way in. Driving in nose-first uses the slot's 340mm against the body's
    150mm WIDTH, which leaves 95mm either side, and the turn itself clears the
    blades by 18mm.

    WHERE IT TURNS IS THE WHOLE THING. A 90-degree turn at full lock drops the
    rear axle 204mm toward the wall and carries it 204mm along it. So the
    follow has to run WIDE - at 450mm out the axle finishes 246mm off the wall
    with the nose 76mm short of it, while at 300mm the nose would finish 74mm
    THROUGH it. And starting the turn level with the first blade lands the
    robot 204mm past it, against a bay half-width of 170: about 30mm past
    centre, with 60mm still to spare on that side.

    NOTHING IS TIMED. Every step ends on a sensor: two on the compass, three
    on the lidar, one on the camera and the lidar together.

    Never touches the motor: update() returns (steer, speed) and the task
    applies it, so the odometry and the status line keep working throughout.
    """

    FOLLOW, CREEP, MEASURE, SETTLE, BACK_CENTRE, SET_TURN, TURN_IN, \
        NOSE_IN, DONE, ABORTED = (
            "follow", "creep", "measure", "settle", "back_centre", "set_turn",
            "turn_in", "nose_in", "done", "aborted")
    DRIVING = (FOLLOW, CREEP, MEASURE, SETTLE, BACK_CENTRE, SET_TURN, TURN_IN,
               NOSE_IN)

    def __init__(self, lidar=None, compass=None, wall_side=1.0,
                 wall_distance_mm=450.0,
                 wall_gain=0.08,
                 wall_max_steer=20.0,
                 side_bearing_deg=90.0,
                 side_sector_deg=30.0,
                 angle_gain=0.6,
                 angle_arc_deg=25.0,
                 angle_min_points=12,
                 angle_max_deg=30.0,
                 front_stop_mm=600.0,
                 front_sector_deg=10.0,
                 front_hold_s=0.4,
                 body_stop_mm=300.0,
                 body_sector_deg=30.0,
                 inner_sector_deg=30.0,
                 inner_slack_mm=250.0,
                 trigger_below_mm=None,
                 mouth_sector_deg=6.0,
                 blade_below_mm=None,
                 lidar_ahead_mm=LIDAR_AHEAD_MM,
                 turn_after_mm=None,
                 measure_bay=True,
                 mouth_clear_mm=60.0,
                 bay_min_mm=170.0,
                 settle_max_mm=600.0,
                 settle_tolerance_mm=40.0,
                 settle_angle_deg=3.0,
                 settle_relax=2.0,
                 creep_max_mm=700.0,
                 turn_in_deg=90.0,
                 turn_in_steer=None,
                 turn_in_min_mm=None,
                 heading_gain=1.0,
                 wall_clean_mm=120.0,
                 wall_heading_blend=0.15,
                 nose_stop_mm=20.0,
                 wheelbase_mm=165.0,
                 max_road_wheel_deg=39.0,
                 vision=None,
                 camera_confirms=True,
                 camera_bearing_deg=60.0,
                 speed=25,
                 reverse_speed=25,
                 servo_settle_s=0.4,
                 mm_per_s_at_full=390.0,
                 nominal_corridor_mm=None,
                 bay_ahead_mm=None,
                 timeout_s=20.0):
        self.lidar = lidar
        self.compass = compass
        # +1 the outer wall is on the robot's right, -1 on its left. Every step
        # below is written in the body's frame - "toward the wall", "away from
        # it" - so this one sign is all that changes between the two.
        self.wall_side = 1.0 if wall_side >= 0 else -1.0
        self.wall_distance_mm = float(wall_distance_mm)
        self.wall_gain = float(wall_gain)
        self.wall_max_steer = float(wall_max_steer)
        self.side_bearing_deg = float(side_bearing_deg)
        self.side_sector_deg = float(side_sector_deg)
        self.angle_gain = float(angle_gain)
        self.angle_arc_deg = float(angle_arc_deg)
        self.angle_min_points = int(angle_min_points)
        self.angle_max_deg = float(angle_max_deg)
        self.front_stop_mm = float(front_stop_mm)
        self.front_sector_deg = float(front_sector_deg)
        self.front_hold_s = float(front_hold_s)
        self.body_stop_mm = float(body_stop_mm)
        self.body_sector_deg = float(body_sector_deg)
        self.inner_sector_deg = float(inner_sector_deg)
        self.inner_slack_mm = float(inner_slack_mm)
        # Default the trigger to "the wall just got 120mm closer", which a
        # 200mm-proud bay wall clears easily and ordinary range noise does not.
        self.trigger_below_mm = (float(trigger_below_mm) if trigger_below_mm
                                 else self.wall_distance_mm - 120.0)
        self.mouth_sector_deg = max(float(mouth_sector_deg), 2.0)
        # The WIDE sector trips the trigger early, because it reports its
        # closest return and so sees a blade off to one side well before the
        # robot is level with it. The NARROW beam is what says "level", and it
        # wants the same threshold.
        self.blade_below_mm = (float(blade_below_mm) if blade_below_mm
                               else self.trigger_below_mm)
        # THE LIDAR IS NOT AT THE AXLE. It sits `lidar_ahead_mm` forward of
        # the rear axle, which moves both ends of this sequence: the beam goes
        # level with a bay wall while the axle is still that far short of it,
        # and it reads the outer wall from that much closer than the nose is.
        # The same number NavigationManager casts its rays from - FinalTask
        # hands it the filter's own lidar_offset_mm unless the config says
        # otherwise, so the two can only disagree on purpose.
        self.lidar_ahead_mm = float(lidar_ahead_mm)
        self.turn_after_mm = _maybe(turn_after_mm)
        self.measure_bay = bool(measure_bay)
        self.mouth_clear_mm = float(mouth_clear_mm)
        self.bay_min_mm = float(bay_min_mm)
        self.settle_max_mm = float(settle_max_mm)
        self.settle_tolerance_mm = float(settle_tolerance_mm)
        self.settle_angle_deg = float(settle_angle_deg)
        # How much the settle gate widens over settle_max_mm of road: 2.0
        # means the tolerances end up three times what they start at. See
        # _settle.
        self.settle_relax = max(0.0, float(settle_relax))
        self.creep_max_mm = float(creep_max_mm)
        self.turn_in_deg = abs(float(turn_in_deg))
        self.turn_in_steer = _maybe(turn_in_steer, absolute=True)
        self.turn_in_min_mm = _maybe(turn_in_min_mm)
        self.heading_gain = float(heading_gain)
        self.wall_clean_mm = float(wall_clean_mm)
        self.wall_heading_blend = float(wall_heading_blend)
        # How far the NOSE stops off the outer wall. Converted to what the
        # front beam will read at that moment, since that is what is measured.
        self.nose_stop_mm = float(nose_stop_mm)
        self.front_target_mm = (self.nose_stop_mm + BODY_FRONT_MM
                                - self.lidar_ahead_mm)
        self.wheelbase_mm = float(wheelbase_mm)
        self.max_road_wheel_deg = float(max_road_wheel_deg)
        self.vision = vision
        self.camera_confirms = bool(camera_confirms)
        self.camera_bearing_deg = float(camera_bearing_deg)
        self.saw_pink = False
        self.speed = int(speed)
        self.reverse_speed = abs(int(reverse_speed))
        self.servo_settle_s = float(servo_settle_s)
        self.mm_per_s_at_full = float(mm_per_s_at_full)
        self.timeout_s = float(timeout_s)

        self.phase = self.FOLLOW
        self.reason = None
        self.side_mm = float("nan")
        self.wall_angle_deg = float("nan")
        self.turned_deg = 0.0
        self.max_steer_seen = 46.0
        self._front_for_s = 0.0
        self._body_for_s = 0.0
        self._level_with_blade = False
        self._level_armed = False
        self._mouth_seen = False
        self._mouth_clear_run_mm = 0.0
        self._beam_lead_mm = 0.0
        self._overshoot_mm = 0.0
        self._rereferenced = False
        self._back_target_mm = 0.0
        self.bay_mm = None              # measured between the two bay walls
        self._warned_camera = False
        # The field heading that runs PARALLEL to the outer wall, learned
        # while following it. What the straight legs steer by once the side
        # lidar is looking at a bay wall instead of at the wall.
        self.wall_heading = None
        # Wall to centre block, learned while the wall side can still be
        # trusted. It is what lets the robot keep its place in the road while
        # it is driving past the bay with the wall side blind.
        # THE ROAD WIDTH IS A RULE, NOT A MEASUREMENT. It starts at the field's
        # own number and the follow refines it; it does not start at None.
        # Learned-only was a silent trap: _learn_corridor is reached through
        # four early returns inside _learn_wall_heading (a NaN fit, an
        # unreadable compass, an unclean arc, an unclean line), and if any of
        # them held for the whole follow the corridor stayed None - at which
        # point _hold_middle quietly returns zero steering and every straight
        # leg after the trigger coasts. From outside that is a robot that
        # stops holding station beside the bay and drifts into it.
        self.nominal_corridor_mm = (float(nominal_corridor_mm)
                                    if nominal_corridor_mm else None)
        self.corridor_mm = self.nominal_corridor_mm
        self.corridor_measured = False
        self._warned_corridor = False
        # HOW FAR THE BAY IS, BY THE LAP COUNTER. The round STARTED in the bay
        # and the counter was zeroed there, so the bay comes back round at a
        # whole number of laps - the robot knows where it is without seeing
        # it. This is that distance, measured from the moment the follow
        # starts, and it is what lets the manoeuvre go ahead when the lidar
        # never recognises the bay: see _follow. None disables the fallback
        # and the lidar is then the only way in.
        self.bay_ahead_mm = (float(bay_ahead_mm) if bay_ahead_mm else None)
        self.triggered_blind = False
        self._fit_clean = False
        self._wall_at_turn = float("nan")
        self._blade_at_mm = 0.0
        self._last_heading = None
        self._warned_no_compass = False
        self.driven_mm = 0.0            # distance in the current leg
        self._elapsed = 0.0
        self._armed = False             # a plausible wall seen at least once



    # ------------------------------------------------------------------
    def summary(self):
        def mm(v):
            return "unset" if v is None else f"{v:.0f}mm"
        side = "right" if self.wall_side > 0 else "left"
        lock = ("full lock" if self.turn_in_steer is None
                else f"{self.turn_in_steer:.0f}")
        return (f"wall on the {side}; hold {self.wall_distance_mm:.0f}mm and "
                f"parallel (angle gain {self.angle_gain:.1f} over a "
                f"{2 * self.angle_arc_deg:.0f}deg arc), trigger under "
                f"{self.trigger_below_mm:.0f}mm"
                f"{' confirmed by camera' if self.camera_confirms else ''}, "
                f"level under {self.blade_below_mm:.0f}mm, "
                f"{mm(self._turn_after())} on, in {self.turn_in_deg:.0f}deg at "
                f"{lock}, nose to {self.nose_stop_mm:.0f}mm "
                f"(front beam {self.front_target_mm:.0f}mm)")

    @property
    def active(self):
        return self.phase in self.DRIVING

    @property
    def finished(self):
        return self.phase in (self.DONE, self.ABORTED)

    def path_caps(self):
        """Nothing: this drives itself from the moment it starts."""
        return (None, None)

    def status_line(self):
        angle = ("--" if math.isnan(self.wall_angle_deg)
                 else f"{self.wall_angle_deg:+.0f}")
        pink = ("" if not self.camera_confirms
                else " pink=y" if self.saw_pink else " pink=n")
        bay = "" if self.bay_mm is None else f" bay={self.bay_mm:.0f}mm"
        measure = (f"turned={self.turned_deg:.0f}deg" if self.phase == self.TURN_IN
                   else f"ahead={self._range_text(self._front_range())}"
                   if self.phase == self.NOSE_IN
                   else f"leg={self.driven_mm:.0f}mm")
        return (f"park {self.phase:8} side={self._range_text(self.side_mm)} "
                f"yaw={angle}{pink}{bay} {measure}")

    # ------------------------------------------------------------------
    def update(self, pose, dt, max_steer=40.0):
        """
        One tick. `pose` is accepted and ignored - deliberately: nothing here
        reads the localizer, which is the entire point.
        """
        if self.finished:
            return None
        self._elapsed += dt
        if self._elapsed > self.timeout_s:
            return self._abort(f"{self.phase} timed out after {self._elapsed:.1f}s")
        self.max_steer_seen = float(max_steer)
        self.side_mm = self._side_range()
        self.wall_angle_deg = self._wall_angle()

        if self.phase == self.FOLLOW:
            return self._follow(dt, max_steer)
        if self.phase == self.CREEP:
            return self._creep(dt, max_steer)
        if self.phase == self.MEASURE:
            return self._measure(dt, max_steer)
        if self.phase == self.SETTLE:
            return self._settle(dt, max_steer)
        if self.phase == self.BACK_CENTRE:
            return self._back_centre(dt, max_steer)
        if self.phase == self.SET_TURN:
            return self._hold(self._turn_in_lock(max_steer), self.TURN_IN)
        if self.phase == self.TURN_IN:
            return self._rotate(dt, self._turn_in_lock(max_steer),
                                self.turn_in_deg, +1, self.NOSE_IN)
        return self._nose_in(max_steer)

    # ------------------------------------------------------------------
    def _creep(self, dt, max_steer):
        """
        Roll on until the axle is level with the first blade, then turn.

        THE NARROW BEAM, not the sector the trigger uses. That sector reports
        its CLOSEST return over 30 degrees, so it catches a blade well before
        the robot is beside it - fine for "a bay is coming", useless for "I am
        level with it". One beam straight out drops only when the blade is
        actually abeam, which is the reference the whole turn is measured from.
        """
        self.driven_mm += self.speed / 100.0 * self.mm_per_s_at_full * dt
        if self._blocked_ahead(dt):
            return self._abort("something in the way beside the bay")
        beam = self._mouth_range()
        # ARM ON THE PLAIN WALL FIRST. The trigger fires off the wide sector,
        # which sees a bay wall well before the robot is level with it - so at
        # the moment the creep starts, the narrow beam should still be reading
        # the plain wall. If it is ALREADY below the threshold the robot is
        # following closer than the threshold, not standing beside a wall, and
        # taking that as "level" turns the robot in a whole bay-width early -
        # straight into the near wall. Wait to see the wall properly first.
        if not math.isnan(beam) and beam > self.blade_below_mm:
            self._level_armed = True
            self._wall_at_turn = beam       # the wall, not the bay wall
        if not self._level_with_blade:
            if self._level_armed and beam < self.blade_below_mm:
                self._level_with_blade = True
                self._blade_at_mm = self.driven_mm
                if self.measure_bay:
                    print("Parking: level with the first bay wall - going on "
                          "to find the second, to measure the bay")
                    self._enter(self.MEASURE)
                    return (self._hold_middle(max_steer),
                            self.speed)
                print(f"Parking: level with the bay wall - "
                      f"{self._turn_after():.0f}mm then turning in")
            elif self.driven_mm >= self.creep_max_mm:
                return self._abort(
                    "rolled past the bay without coming level with a wall"
                    if self._level_armed else
                    f"the side beam never rose above {self.blade_below_mm:.0f}mm "
                    f"- following too close to tell a bay wall from the wall")
            return (self._hold_middle(max_steer), self.speed)
        if self.driven_mm - self._blade_at_mm >= self._turn_after():
            room = self._turn_in_room()
            if not math.isnan(self._wall_at_turn) and self._wall_at_turn < room:
                # THE TURN CANNOT CLEAR FROM HERE, so do not start it. A
                # 90-degree turn at full lock carries the axle a whole radius
                # toward the wall before the nose comes round, so the axle has
                # to start at least radius + nose away from it. Attempted from
                # closer, the body is through a bay wall before it is halfway
                # round - at 360mm out it clips by 40mm, at 320mm the nose
                # ends up 54mm INSIDE the outer wall. Better to give the
                # attempt up and let the retry have another go from further
                # out than to turn into something.
                return self._abort(
                    f"only {self._wall_at_turn:.0f}mm off the wall at the bay "
                    f"and the turn needs {room:.0f}mm - follow further out")
            self._enter(self.SET_TURN)
            return (0.0, 0)
        return (self._hold_middle(max_steer), self.speed)

    def _measure(self, dt, max_steer):
        """
        Carry on past the first bay wall until the second one is abeam.

        MEASURED, NOT ASSUMED. Knowing where BOTH walls are gives the bay's
        real width and therefore its real middle, so the turn is aimed at the
        bay in front of the robot rather than at a nominal 340mm one. It costs
        a bay-length of extra travel and the reverse that undoes it, which is
        the whole reason for backing up at all.

        The beam has to come clear of the first wall before a second drop
        counts - otherwise the first wall, still abeam, reads as the second.
        """
        step = self.speed / 100.0 * self.mm_per_s_at_full * dt
        self.driven_mm += step
        if self._blocked_ahead(dt):
            return self._abort("something in the way past the first bay wall")
        beam = self._mouth_range()
        # THE MOUTH HAS TO STAY OPEN. A bay wall is 10mm thick and the beam's
        # footprint is wider than that, so right at the crossing it flickers
        # clear-then-blocked within 20mm - which reads as a 20mm-wide bay and
        # sends the robot back to the middle of nothing. Only a run of clear
        # counts as the mouth, and only a second wall at a credible distance
        # counts as the far side.
        if not math.isnan(beam) and beam > self.blade_below_mm:
            self._mouth_clear_run_mm += step
            if self._mouth_clear_run_mm >= self.mouth_clear_mm:
                self._mouth_seen = True
        else:
            self._mouth_clear_run_mm = 0.0
        if (self._mouth_seen and self.driven_mm >= self.bay_min_mm
                and not math.isnan(beam) and beam < self.blade_below_mm):
            self.bay_mm = self.driven_mm
            # THE BEAM SEES EACH WALL EARLY. It is a few degrees wide, so the
            # wall enters the cone before the lidar is level with it - by
            # range x tan(half-angle), about 13mm at this range. That cancels
            # out of the WIDTH, since both walls are caught the same amount
            # early, but not out of where the robot actually is: the axle is
            # that much further back than the arithmetic assumes.
            self._beam_lead_mm = beam * math.tan(
                math.radians(self.mouth_sector_deg / 2.0))
            print(f"Parking: bay measured at {self.bay_mm:.0f}mm wide "
                  f"(nominal {NOMINAL_BAY_MM:.0f}) - squaring up on the wall "
                  f"past it before backing in")
            self._enter(self.SETTLE)
            return (self._hold_middle(max_steer), self.speed)
        if self.driven_mm >= self.creep_max_mm:
            # Never found the far wall. Fall back on the nominal width by
            # reversing to where turn_after_mm would have stopped.
            self.bay_mm = NOMINAL_BAY_MM
            self._beam_lead_mm = 0.0
            self._overshoot_mm = self.driven_mm - NOMINAL_BAY_MM
            print(f"Parking: second bay wall not seen in "
                  f"{self.driven_mm:.0f}mm - using the nominal width")
            self._enter(self.SETTLE)
            return (self._hold_middle(max_steer), self.speed)
        return (self._hold_middle(max_steer), self.speed)

    def _settle(self, dt, max_steer):
        """
        Get square and back out to the follow distance BEFORE reversing.

        Everything after this is open loop - a measured reverse, then a turn
        on the compass, then a straight run in - so wherever the robot is when
        it starts backing up is where the whole manoeuvre is measured from.
        And the place it would otherwise start from is the worst one
        available: alongside the bay, where the side lidar has been looking at
        a bay wall rather than the wall, so the follow has been flying blind
        for a bay's length and whatever drift it picked up there is baked in.

        Past the far wall the beam is back on clean wall and the follow works
        again, so it is given a little road to straighten out on. The distance
        it spends doing that is added to the reverse, so squaring up does not
        move where the robot ends up.
        """
        step = self.speed / 100.0 * self.mm_per_s_at_full * dt
        self.driven_mm += step
        if self._blocked_ahead(dt):
            return self._abort("something in the way while squaring up")
        if self._corner_ahead(dt):
            return self._abort("ran out of wall to square up on before the bay")
        self._learn_wall_heading()      # the arc is clean again out here
        clean = self._arc_min_range()
        # THE BAR COMES DOWN AS THE ROAD RUNS OUT. Fixed tolerances make this
        # all or nothing: either the follow gets inside 40mm and 3 degrees, or
        # the robot walks the full settle_max_mm forward and backs in from
        # wherever it happened to be - and on a slow-settling run that is
        # hundreds of millimetres of extra ground covered for nothing, which
        # is what "it sets up too slow and just keeps walking forward" is.
        # Widening the gate with distance takes the first pose that is good
        # enough for the road left, so a 20mm-off robot stops at once and only
        # a genuinely crooked one uses the whole allowance.
        slack = min(1.0, self.driven_mm / max(1.0, self.settle_max_mm)) * self.settle_relax
        tolerance = self.settle_tolerance_mm * (1.0 + slack)
        angle = self.settle_angle_deg * (1.0 + slack)
        square = (not math.isnan(clean)
                  and abs(clean - self.wall_distance_mm) < tolerance
                  and not math.isnan(self.side_mm)
                  and abs(self.side_mm - self.wall_distance_mm) < tolerance
                  and not math.isnan(self.wall_angle_deg)
                  and abs(self.wall_angle_deg) < angle)
        print(f"{self.driven_mm:.0f} side={self.side_mm:.0f} angle={self.wall_angle_deg:.1f} clean={clean:.0f} sq={square}")
        if square or self.driven_mm >= self.settle_max_mm:
            if not square:
                print(f"Parking: never settled in {self.driven_mm:.0f}mm "
                      f"(side {self._range_text(self.side_mm)}, yaw "
                      f"{self.wall_angle_deg:+.0f}) - backing in anyway")
            self._back_target_mm = (self.bay_mm / 2.0
                                    + self._turn_radius(self.turn_in_steer
                                                        or self.max_steer_seen)
                                    - self.lidar_ahead_mm - self._beam_lead_mm
                                    + self.driven_mm + self._overshoot_mm)
            print(f"Parking: square at {self._range_text(self.side_mm)} off the "
                  f"wall - backing up {self._back_target_mm:.0f}mm to the "
                  f"middle of the bay")
            self._enter(self.BACK_CENTRE)
            return (0.0, 0)
        return (self._track(self.wall_distance_mm, max_steer), self.speed)

    def _back_centre(self, dt, max_steer):
        """
        Reverse to the point the turn has to start from.

        THE CORRECTION IS INVERTED GOING BACKWARDS. A steering angle that
        turns the nose one way going forward turns it the other way in
        reverse, so the heading hold has to be negated here or it drives the
        error it is trying to remove.
        """
        # RE-REFERENCE OFF THE WALL ON THE WAY BACK. Reversing is dead
        # reckoning, and the settle can add a couple of hundred millimetres to
        # it - which is why squaring up cost more centring than it bought
        # (measured: 12mm off centre without it, 34mm with). But the far bay
        # wall is passed again on the way back, and the beam sees it: from
        # there the distance left is a fixed sum, so however far the settle
        # went, only that last stretch is reckoned rather than measured.
        beam = self._mouth_range()
        if (not self._rereferenced and self.driven_mm > self.mouth_clear_mm
                and not math.isnan(beam) and beam < self.blade_below_mm
                and self.bay_mm is not None):
            self._rereferenced = True
            # The lead flips: going backwards the wall enters the cone once
            # the lidar is already PAST it, so it adds to what is left.
            lead = beam * math.tan(math.radians(self.mouth_sector_deg / 2.0))
            left = (self.bay_mm / 2.0
                    + self._turn_radius(self.turn_in_steer or self.max_steer_seen)
                    + lead - self.lidar_ahead_mm)
            self._back_target_mm = self.driven_mm + left
            print(f"Parking: bay wall abeam again on the way back - "
                  f"{left:.0f}mm left to the middle")
        want = abs(self._back_target_mm)
        forwards = self._back_target_mm < 0.0
        if self.driven_mm >= want:
            self._enter(self.SET_TURN)
            return (0.0, 0)
        speed = self.speed if forwards else self.reverse_speed
        self.driven_mm += speed / 100.0 * self.mm_per_s_at_full * dt
        steer = self._hold_middle(max_steer)
        return (steer if forwards else -steer,
                int(speed if forwards else -speed))

    def _nose_in(self, max_steer):
        """
        Straight into the bay until the outer wall is `nose_stop_mm` off.

        Measured on the front lidar, so it stops the same distance off the
        wall whatever the turn left behind it - the one place in the sequence
        where an error in everything upstream gets absorbed rather than added.
        """
        ahead = self._front_range()
        if ahead <= self.front_target_mm:
            print(f"Parking: in the bay, nose about "
                  f"{ahead - BODY_FRONT_MM + self.lidar_ahead_mm:.0f}mm off "
                  f"the outer wall")
            self._enter(self.DONE)
            return (0.0, 0)
        return (self._hold_heading(self._bay_heading(), max_steer), self.speed)

    def _rotate(self, dt, steer, degrees, direction, then):
        """
        Hold a lock until the body has come round `degrees`, on the compass.

        Summed tick by tick rather than compared against the start, so passing
        180 does not wrap back to nothing. Reversing negates the yaw rate,
        which is the whole trick of the last step: the same lock that turned
        the robot into the bay turns it back out flat when it reverses on it.
        """
        speed = self.speed if direction > 0 else -self.reverse_speed
        heading = self._heading()
        if heading is None:
            if not self._warned_no_compass:
                self._warned_no_compass = True
                print("Parking: no compass - turning on the arc length instead")
            arc = math.radians(degrees) * self._turn_radius(steer)
            if self.driven_mm >= arc:
                self._enter(then)
                return (steer, 0)
            self.driven_mm += abs(speed) / 100.0 * self.mm_per_s_at_full * dt
            return (steer, int(speed))
        if self._last_heading is not None:
            self.turned_deg += abs(angle_difference(heading, self._last_heading))
        self._last_heading = heading
        if self.turned_deg >= degrees:
            self._enter(then)
            return (steer, 0)
        return (steer, int(speed))

    def _turn_radius(self, steer):
        """Rear-axle radius at a steering command, for the no-compass fallback."""
        wheel = math.radians(abs(steer) / max(self.max_steer_seen, 1.0)
                             * self.max_road_wheel_deg)
        return self.wheelbase_mm / math.tan(wheel) if wheel > 1e-6 else 1e9

    def _turn_in_room(self):
        """
        The least distance off the wall the turn-in can be started from.

        The radius the axle swings through, plus the body ahead of the axle,
        plus a little - below that the nose is through the outer wall before
        the turn finishes, whatever else is set.
        """
        if self.turn_in_min_mm is not None:
            return self.turn_in_min_mm
        steer = self.turn_in_steer or self.max_steer_seen
        return self._turn_radius(steer) + BODY_FRONT_MM + 30.0

    def _turn_after(self):
        """
        How much further to roll past the bay wall before turning in.

        Unset means "land the body in the middle of the bay", which is a sum
        of three lengths and no tuning: half the bay, plus how far the lidar
        leads the axle (the beam went level with the wall while the axle was
        still short of it), minus the radius the turn carries the axle
        through. At 170 + 150 - 204 that is about 116mm - and at zero the robot
        lands 116mm off centre, which is enough to graze the far wall.
        """
        if self.turn_after_mm is not None:
            return self.turn_after_mm
        steer = self.turn_in_steer or self.max_steer_seen
        return (NOMINAL_BAY_MM / 2.0 + self.lidar_ahead_mm
                - self._turn_radius(steer))

    def _turn_in_lock(self, max_steer):
        return self._lock(self.turn_in_steer or max_steer, toward=True,
                          max_steer=max_steer)

    def _heading(self):
        if self.compass is None:
            return None
        try:
            return self.compass.heading()
        except Exception:
            return None

    def _follow(self, dt, max_steer):
        """
        Drive alongside the wall at `wall_distance_mm`, watching for the step.

        TWO TERMS, NOT ONE. A follow that only sees a distance cannot tell a
        robot that is 250mm out and parallel from one that is 250mm out and
        crabbing at 15 degrees: both report 250, both get steering zero, and
        the second one drives straight on whatever heading pure pursuit left
        it holding until the range error finally builds. That is what the
        wheels did on the run in - freeze the heading at the handover and walk
        forward. So the angle to the wall is measured as well, and corrected,
        on every tick right up to the bay wall.
        """
        self.driven_mm += self.speed / 100.0 * self.mm_per_s_at_full * dt
        # THE COUNTER KNOWS WHERE THE BAY IS EVEN WHEN THE LIDAR DOES NOT.
        # The round started inside the bay and the counter was zeroed there,
        # so it comes back round at a whole number of laps. When the follow
        # has driven the distance that says the bay is here and the side beam
        # still has not recognised it, turn in anyway: the run-up has already
        # been spent settling onto the wall, so the robot IS beside the bay -
        # it is only the recognition that failed, and driving past a bay that
        # is definitely there is the worse of the two mistakes.
        #
        # STRAIGHT TO THE TURN, not to the creep. The creep's whole job is to
        # find the near blade on the narrow beam and measure the turn point
        # from it; with nothing to see it would roll to creep_max_mm and abort.
        # The counter gives the same answer directly - and the turn is started
        # one radius SHORT of the bay's middle, because a 90-degree turn at
        # full lock carries the axle about that far along the wall as it comes
        # round.
        if self.bay_ahead_mm is not None and not self.triggered_blind:
            radius = self._turn_radius(self._turn_in_lock(max_steer))
            if self.driven_mm >= max(0.0, self.bay_ahead_mm - radius):
                self.triggered_blind = True
                side = self.side_mm if not math.isnan(self.side_mm) else None
                print(f"Parking: no bay wall recognised in {self.driven_mm:.0f}mm, "
                      f"but the lap counter puts the bay here - turning in on the "
                      f"counter (side {self._range_text(self.side_mm)}, wanted under "
                      f"{self.trigger_below_mm:.0f}"
                      + ("" if not self.camera_confirms else
                         f"; pink {'seen' if self.saw_pink else 'never seen'}")
                      + f"; armed={self._armed}).")
                # The same clearance guard the creep applies. Turning in from
                # too close puts the body through a bay wall before it is
                # halfway round, and that is true however the turn was
                # triggered.
                room = self._turn_in_room()
                if side is not None and side < room:
                    return self._abort(
                        f"the counter says the bay is here but the robot is only "
                        f"{side:.0f}mm off the wall and the turn needs {room:.0f}mm "
                        f"- follow further out (parking.wall_distance_mm)")
                self._wall_at_turn = side if side is not None else self.wall_distance_mm
                self.bay_mm = NOMINAL_BAY_MM
                self._enter(self.SET_TURN)
                return (0.0, 0)
        if math.isnan(self.side_mm):
            return (0.0, self.speed)            # blind for a tick; keep rolling
        # ARM ON THE PLAIN WALL, AT THE RIGHT DISTANCE FROM IT. Merely seeing
        # SOMETHING is not enough: a robot that starts the approach closer to
        # the wall than the trigger threshold reads below it on the very first
        # tick, so it arms and triggers together and marches into the
        # manoeuvre without the follow ever running. From the outside that is
        # a robot driving straight past the middle of the road without
        # appearing to try. Being at the follow distance is what says the
        # range means the wall, and only then can a step down mean a bay.
        # AND WITH A GAP BETWEEN ARMING AND TRIGGERING. Arming at "within
        # wall_clean_mm" put the two thresholds back to back - the robot
        # crossed one on its way up and dipped back through the other on
        # noise, and called 5mm of wobble a bay wall. Arming means SETTLED on
        # the wall, which leaves a clear step between here and the trigger.
        if abs(self.side_mm - self.wall_distance_mm) < self.settle_tolerance_mm:
            self._armed = True
        self._learn_wall_heading()
        # ASK THE CAMERA EVERY TICK, not at the trigger. The camera looks
        # FORWARD: it can see the bay from a couple of metres back, and by the
        # time the side lidar trips - the bay abeam - the pink is 66 degrees
        # off the nose and long out of frame. Checking only at the trigger
        # meant it was always asked at the one moment it could never answer.
        self._pink_ahead()
        if self._blocked_ahead(dt):
            return self._abort("something in the way on the approach - "
                               "handing back so the lap can drive round it")
        if self._corner_ahead(dt):
            # Something across the road with the bay still not found: the end
            # of this wall, or a traffic sign standing in it. Either way the
            # approach cannot continue, and the difference does not matter -
            # stop, hand back, and let the lap drive round it.
            return self._abort(
                f"blocked {self._range_text(self._front_range())} ahead without "
                f"finding the bay - handing back so the lap can drive round it"
                + ("" if self._armed else
                   f" (and it never settled to {self.wall_distance_mm:.0f}mm "
                   f"off the wall, so the bay could not have been recognised "
                   f"anyway)"))
        if self._armed and self.side_mm < self.trigger_below_mm:
            if self.camera_confirms and not self.saw_pink:
                # The lidar sees SOMETHING 200mm proud of the wall. Only the
                # camera can say it is pink, and a step this size that is not
                # pink is a pillar sitting near the wall - which the lap is
                # meant to drive round, not park in. Keep following.
                #
                # SAY SO, ONCE. Holding off here is indistinguishable from the
                # bay never being found: the robot drives past a bay it can
                # see perfectly well on the lidar and gives no reason at all.
                if not self._warned_camera:
                    self._warned_camera = True
                    print(f"Parking: lidar has a bay wall at "
                          f"{self.side_mm:.0f}mm but the camera has not seen "
                          f"pink, so the trigger is being held off. If that "
                          f"is all it ever says, either Color.PINK does not "
                          f"match the real walls (check it with "
                          f"test_color_picker.py) or set "
                          f"parking.camera_confirms = false to run on the "
                          f"lidar alone.")
                return (self._track(self.wall_distance_mm, max_steer), self.speed)
            print(f"Parking: bay wall at {self.side_mm:.0f}mm on the "
                  f"{'right' if self.wall_side > 0 else 'left'}"
                  f"{', pink confirmed' if self.saw_pink else ''}")
            # EVERYTHING FROM HERE HOLDS STATION ON THESE TWO NUMBERS, and
            # neither is measurable any more once the side beam is on a bay
            # wall. Say plainly which of them the follow actually got, because
            # missing either is the difference between holding the middle of
            # the road and coasting into the bay - and it used to be silent.
            print(f"  corridor {self.corridor_mm:.0f}mm "
                  f"({'measured' if self.corridor_measured else 'ASSUMED - the '
                     'follow never got a clean look at both sides'}); "
                  f"wall heading "
                  + ("unknown - THE STRAIGHT LEGS WILL NOT HOLD A HEADING"
                     if self.wall_heading is None
                     else f"{self.wall_heading:.0f}deg"))
            self._enter(self.CREEP)
            return (0.0, self.speed)
        return (self._track(self.wall_distance_mm, max_steer), self.speed)

    def _learn_wall_heading(self):
        """
        Remember which way the wall runs, in field degrees.

        Only the follow can work this out: it is the one phase where the side
        lidar is looking at the outer wall, so the fitted yaw means something.
        The moment a bay wall enters that beam the measurement is gone - which
        is exactly when the straight legs start needing it - so it is banked
        here and held.
        """
        if math.isnan(self.wall_angle_deg):
            return
        heading = self._heading()
        if heading is None:
            return
        # ONLY OFF CLEAN WALL, AND AVERAGED. Two traps here, and the naive
        # version walks into both. The fit is worth about +-2.5 degrees tick
        # to tick, so the LAST reading is a poor answer - and the last reading
        # is the one taken as a bay wall slides into the arc the line is
        # fitted through, which is the most corrupted of the lot. Measured: it
        # dragged the estimate from a true 180 to 188 over the last half
        # second, and the creep then dutifully steered to 188.
        #
        # So stop the moment the FIT says a point does not belong - which is
        # about 90mm before the side range notices anything - and average what
        # came before rather than trust any single tick.
        # NOTHING ELSE IN THE ARC. Rejected points are not enough on their
        # own: a bay wall is a flat face, so once enough of the arc lands on
        # it the fit is a perfectly clean line - just the wrong one, with no
        # outliers to notice. What gives it away is that something in the arc
        # is far closer than the wall being followed. Measured: without this
        # the estimate sat on a true 180 for the whole follow and then went to
        # 187 in the last half second, and the creep steered to 187.
        arc = self._arc_min_range()
        if math.isnan(arc) or arc < self.wall_distance_mm - self.wall_clean_mm:
            return
        if not self._fit_clean:
            return
        # The yaw is + when the nose is turned AWAY from the wall, and which
        # way round that is on a compass depends on the side the wall is on.
        self._learn_corridor()
        seen = normalize_angle(heading + self.wall_angle_deg * self.wall_side)
        if self.wall_heading is None:
            self.wall_heading = seen
            return
        self.wall_heading = normalize_angle(
            self.wall_heading
            + self.wall_heading_blend * angle_difference(seen, self.wall_heading))

    def _arc_min_range(self):
        """Closest return anywhere in the arc the wall angle is fitted through."""
        if self.lidar is None:
            return float("nan")
        aimed = self.side_bearing_deg * self.wall_side
        lo, hi = aimed - self.angle_arc_deg, aimed + self.angle_arc_deg
        distance, _ = self.lidar.get_min_distance(min(lo, hi), max(lo, hi))
        if distance is None or math.isnan(distance):
            return float("nan")
        return float(distance)

    def _inner_range(self):
        """Distance to the centre block - the wall on the far side of the road."""
        if self.lidar is None:
            return float("nan")
        half = self.inner_sector_deg / 2.0
        aimed = -self.side_bearing_deg * self.wall_side
        distance, _ = self.lidar.get_min_distance(aimed - half, aimed + half)
        if distance is None or math.isnan(distance):
            return float("nan")
        return float(distance)

    def _learn_corridor(self):
        """
        How wide the road is here, measured while both sides can be seen.

        Banked for the same reason as the wall's heading: it is only knowable
        while the side lidar is on clean wall, and it is only NEEDED once it
        is not.
        """
        inner = self._inner_range()
        if math.isnan(inner) or math.isnan(self.side_mm):
            return
        seen = self.side_mm + inner
        # IT CANNOT BE WIDER THAN THE ROAD IS. Both terms are the CLOSEST
        # return over a sector, so a robot that is not square to the wall
        # inflates each of them by about 1/cos(yaw) - the error only ever goes
        # one way, and the sum can only ever come out too BIG. Left unclamped
        # that overestimate is poison, because _hold_middle turns it straight
        # into a wall distance: a corridor read 124mm wide holds the robot
        # 124mm CLOSER to the wall than it was told to, which is how the
        # approach arrives at the bay far too close to square up and aborts
        # on "ran out of wall". Measured on the robot: 1124mm across a road
        # that is 1000mm wide, and the settle started 231mm off a wall it was
        # supposed to be 450mm off.
        if self.nominal_corridor_mm is not None:
            if seen > self.nominal_corridor_mm and not self._warned_corridor:
                self._warned_corridor = True
                print(f"Parking: the road measured {seen:.0f}mm across but it is "
                      f"{self.nominal_corridor_mm:.0f}mm - the robot is not square "
                      f"to the wall, so both ranges read long. Using "
                      f"{self.nominal_corridor_mm:.0f}.")
            seen = min(seen, self.nominal_corridor_mm)
        self.corridor_mm = (seen if self.corridor_mm is None
                            else self.corridor_mm
                            + self.wall_heading_blend * (seen - self.corridor_mm))
        self.corridor_measured = True

    def _hold_middle(self, max_steer):
        """
        Keep station in the road while the wall side is looking at the bay.

        THE OTHER WALL DOES THE WORK. Alongside the bay the side lidar reads a
        bay wall, not the wall - a follow built on it would see 250mm, decide
        it was too close and steer AWAY from the very thing it is trying to
        park in. So the centre block is used instead: the road's width was
        measured on the way in, so the distance to the block that corresponds
        to the right distance from the wall is known, and the robot holds
        THAT. Heading comes from the compass on top, as before.

        Falls back to heading alone when the block is not where it should be -
        near a corner it is not there at all, and a beam that sees across the
        field instead would drag the robot into the wall.
        """
        steer = self._hold_heading(self.wall_heading, max_steer)
        if self.corridor_mm is None:
            return steer
        inner = self._inner_range()
        if math.isnan(inner):
            return steer
        want = self.corridor_mm - self.wall_distance_mm
        if abs(inner - want) > self.inner_slack_mm:
            return steer            # not the block: a corner, or a gap past it
        # Too far from the block is too close to the wall, so it wants the
        # opposite sign to the ordinary follow's "too far out".
        push = clamp(self.wall_gain * (want - inner),
                     -self.wall_max_steer, self.wall_max_steer)
        return clamp(steer + self.wall_side * push, -max_steer, max_steer)

    def _hold_heading(self, target_deg, max_steer):
        """
        Steering that keeps an absolute compass heading.

        For the legs that are meant to be straight. Steering zero is not the
        same as going straight: these legs run on after a turn, from a servo
        that has just been at full lock, and two degrees held for half a metre
        is the difference between arriving square in the bay and across it.
        """
        if target_deg is None or self.heading_gain <= 0.0:
            return 0.0
        heading = self._heading()
        if heading is None:
            return 0.0
        steer = clamp(self.heading_gain * angle_difference(target_deg, heading),
                      -self.wall_max_steer, self.wall_max_steer)
        return clamp(steer, -max_steer, max_steer)

    def _bay_heading(self):
        """Square to the outer wall - 90 degrees off the wall's own heading."""
        if self.wall_heading is None:
            return None
        return normalize_angle(self.wall_heading + 90.0 * self.wall_side)
        
    def _track(self, target_mm, max_steer):
        """
        The steering that holds `target_mm` off the wall AND holds it parallel.
        ...
        """
        if math.isnan(self.side_mm) and math.isnan(self.wall_angle_deg):
            return 0.0
        steer = 0.0 if math.isnan(self.side_mm) else self.wall_gain * (self.side_mm - target_mm)
        if not math.isnan(self.wall_angle_deg):
            steer += self.angle_gain * self.wall_angle_deg
        steer = clamp(steer, -self.wall_max_steer, self.wall_max_steer)
        return clamp(self.wall_side * steer, -max_steer, max_steer)

    def _hold(self, steer, then):
        """Stand still while the servo travels to a new angle."""
        if self._elapsed >= self.servo_settle_s:
            self._enter(then)
        return (steer, 0)

    def _lock(self, magnitude, toward, max_steer):
        """
        A steering magnitude, signed.

        `toward` points the wheels at the wall. That is the FIRST reverse leg,
        the same way a driver starts a parallel park: reversing on it swings
        the tail into the space and carries the axle in toward the wall. The
        second leg is the counter-lock, which brings the nose round and
        straightens the body up in the space.
        """
        sign = self.wall_side if toward else -self.wall_side
        return clamp(sign * magnitude, -max_steer, max_steer)

    def _wall_angle(self):
        """
        Yaw relative to the wall in degrees, + = nose turned away from it, or
        nan when the arc does not look like a wall.

        A LINE FIT, NOT TWO RAYS. Two ranges either side of the perpendicular
        give the angle in one line of trigonometry, and at this range they
        give it terribly: the C1 is good to about a centimetre, and a
        centimetre of error across a 20-degree baseline at 250mm is eight
        degrees of yaw. Eight degrees times the gain is a steering command
        that weaves on noise alone. Fitting a line through the whole visible
        side arc instead averages fifty of them down to about two.

        THE ARC STOPS SHORT OF THE BLIND SPOT. Past USABLE_FOV_DEG the scan
        returns the robot's own body at ~110mm - which reads as a wall
        crashing in at 45 degrees, and is why the first version of this
        measured nothing at all in the sim: its aft ray sat exactly on the
        edge. The arc is clamped there rather than trusted to the config.
        """
        if self.lidar is None or self.angle_gain <= 0.0:
            return float("nan")
        points = self._side_points()
        if len(points) < self.angle_min_points:
            return float("nan")
        line = self._fit_line(points)
        if line is None:
            return float("nan")
        # One robust pass: the bay wall enters the front of the arc before it
        # reaches the perpendicular beam, and a handful of points 200mm proud
        # would drag the fit round. Throw the outliers out and fit again.
        kept = self._without_outliers(points, line)
        if len(kept) < self.angle_min_points:
            return float("nan")
        # WHETHER ANYTHING WAS THROWN OUT IS ITSELF A MEASUREMENT: it is the
        # first sign that something which is not the outer wall has entered
        # the arc. The arc reaches about 210mm ahead where the side sector
        # only reaches 120mm, so this notices a bay wall roughly 90mm - half a
        # second - before the range does. _learn_wall_heading needs that
        # warning, because those are exactly the ticks that would poison it.
        self._fit_clean = len(kept) == len(points)
        if len(kept) < len(points):
            line = self._fit_line(kept)
            if line is None:
                return float("nan")
        yaw = math.degrees(math.atan(line[0]))
        if abs(yaw) > self.angle_max_deg:
            return float("nan")     # a corner or a step, not the wall
        return yaw

    def _side_points(self):
        """
        The visible arc of wall as (along, depth) millimetres.

        Mirrored into the frame where the wall is on the right and the nose
        points along +along, so everything downstream is written once and the
        wall side only appears when the scan is indexed.
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
            if math.isnan(distance):
                continue
            if not MIN_DETECT_RANGE_MM < distance < MAX_DETECT_RANGE_MM:
                continue
            radians = math.radians(bearing)
            points.append((distance * math.cos(radians),
                           distance * math.sin(radians)))
        return points

    @staticmethod
    def _fit_line(points):
        """
        Least squares depth = intercept + slope * along, or None.

        With the wall on the right and the robot yawed by psi, every return
        satisfies depth = D/cos(psi) + along*tan(psi) exactly - so the slope
        IS the tangent of the yaw, and the depth cancels out of it.
        """
        count = len(points)
        mean_along = sum(p[0] for p in points) / count
        mean_depth = sum(p[1] for p in points) / count
        spread = sum((p[0] - mean_along) ** 2 for p in points)
        if spread < 100.0:              # all bunched up: no baseline, no angle
            return None
        cross = sum((p[0] - mean_along) * (p[1] - mean_depth) for p in points)
        slope = cross / spread
        return slope, mean_depth - slope * mean_along

    @staticmethod
    def _without_outliers(points, line):
        slope, intercept = line
        residuals = [p[1] - (slope * p[0] + intercept) for p in points]
        rms = math.sqrt(sum(r * r for r in residuals) / len(residuals))
        limit = max(3.0 * rms, 40.0)
        return [p for p, r in zip(points, residuals) if abs(r) <= limit]

    def _mouth_range(self):
        """
        One narrow beam straight out at the wall, or nan.

        Deliberately not `side_mm`, which is the closest return over a
        30-degree sector: from inside the bay mouth the blades are still in
        that sector either side, so it goes on reading 50mm until the robot is
        clear of the whole bay - exactly too late to stop in it. A few degrees
        wide, it sees between them.
        """
        if self.lidar is None:
            return float("nan")
        half = self.mouth_sector_deg / 2.0
        aimed = self.side_bearing_deg * self.wall_side
        distance, _ = self.lidar.get_min_distance(aimed - half, aimed + half)
        if distance is None or math.isnan(distance):
            return float("nan")
        return float(distance)

    def _pink_ahead(self):
        """
        Is the camera looking at a bay wall, on the side the lidar says?

        BOTH SENSORS, EACH FOR WHAT IT IS GOOD AT. The lidar knows exactly how
        far away the step is and nothing about its colour; the camera knows
        the colour and, through a bay wall of unknown size, very little about
        the range. So the lidar picks the moment and the camera only has to
        agree that the thing over there is pink and roughly on the right side.

        Once it has agreed, it stays agreed - and it has to, because the mark
        leaves the frame long before the robot is level with it. This is
        called on every tick of the follow so the latch is set while the bay
        is still ahead, not at the trigger, when it is already abeam.
        """
        if self.vision is None:
            self.saw_pink = True        # nothing to ask; the lidar stands alone
            return True
        if self.saw_pink:
            return True
        try:
            marks = self.vision.parking_marks()
        except Exception:
            return True                 # camera trouble must not block the park
        for mark in marks:
            # + bearing is to the robot's right, and so is wall_side +1.
            if mark.bearing_deg * self.wall_side > -self.camera_bearing_deg:
                self.saw_pink = True
                return True
        return False

    def _blocked_ahead(self, dt):
        """
        Is something the BODY would hit sitting in front of the robot?

        PURE PURSUIT IS OFF FROM THE MOMENT PARKING STARTS, and the pillar
        dodging goes with it. Nothing else is watching where the approach is
        going, so every phase that drives forward has to watch for itself -
        otherwise a traffic sign standing anywhere along it gets driven into
        without the robot ever trying to avoid it.

        Wider and nearer than the corner check: 30 degrees covers +-80mm at
        300mm ahead, which is the body's own width, while the wall being
        followed is 1700mm away down that ray and a bay wall 970mm, so neither
        can trip it. The corner check is narrow and far instead, because a
        corner is a wall across the whole road rather than a thing in it.
        """
        if self.lidar is None:
            return False
        half = self.body_sector_deg / 2.0
        distance, _ = self.lidar.get_min_distance(-half, half)
        near = (distance is not None and not math.isnan(distance)
                and distance < self.body_stop_mm)
        self._body_for_s = self._body_for_s + dt if near else 0.0
        return self._body_for_s >= self.front_hold_s

    def _corner_ahead(self, dt):
        """
        Is there a wall across the path, as opposed to one alongside it?

        NARROW, AND IT HAS TO PERSIST. A wide sector reads the outer wall as
        an obstacle the moment the robot is angled at all - at 250mm out and
        10 degrees of nose-in, a beam 20 degrees off the nose meets the wall
        it is following 570mm ahead, which is indistinguishable from a corner
        by range alone. Ten degrees of sector needs 20 degrees of yaw before
        it can be fooled, and the follow does not hold that; the hold time
        then covers the transient, because a corner keeps getting closer while
        a yaw excursion corrects itself.
        """
        if self._front_range() < self.front_stop_mm:
            self._front_for_s += dt
        else:
            self._front_for_s = 0.0
        return self._front_for_s >= self.front_hold_s

    def _front_range(self):
        """Closest return dead ahead, or +inf when nothing is in the way."""
        if self.lidar is None:
            return float("inf")
        half = self.front_sector_deg / 2.0
        distance, _ = self.lidar.get_min_distance(-half, half)
        if distance is None or math.isnan(distance):
            return float("inf")
        return float(distance)

    def _side_range(self):
        """Closest return in the sector facing the wall, or nan."""
        if self.lidar is None:
            return float("nan")
        half = self.side_sector_deg / 2.0
        bearing = self.side_bearing_deg * self.wall_side
        distance, _ = self.lidar.get_min_distance(bearing - half, bearing + half)
        return float("nan") if distance is None or math.isnan(distance) else float(distance)

    def _enter(self, phase):
        self.phase = phase
        self._elapsed = 0.0
        self.driven_mm = 0.0
        self.turned_deg = 0.0
        self._front_for_s = 0.0
        self._body_for_s = 0.0
        self._last_heading = None

    def _abort(self, reason):
        self.phase = self.ABORTED
        self.reason = reason
        print(f"Parking aborted: {reason}")
        return (0.0, 0)

    @staticmethod
    def _range_text(value):
        return "--" if math.isnan(value) else f"{value:.0f}mm"


def _maybe(value, absolute=False):
    """A configured number, or None when it has not been measured yet."""
    if value is None:
        return None
    return abs(float(value)) if absolute else float(value)

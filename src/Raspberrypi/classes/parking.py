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

from utils.angle_utils import angle_difference, clamp

# ============================================================================
# The bay, as the rules build it
# ============================================================================
WALL_LENGTH_MM = 200.0      # how far each wall sticks out from the outer wall
WALL_THICKNESS_MM = 10.0    # its extent ALONG the outer wall
WALL_HEIGHT_MM = 100.0      # low enough that a high lidar mount misses it
NOMINAL_BAY_MM = 300.0      # clear gap between the two inner faces
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
                 min_gap_mm=250.0, max_gap_mm=400.0, min_scans=3):
        self.map = field_map
        self.section = section
        self.min_depth_mm = float(min_depth_mm)
        self.min_gap_mm = float(min_gap_mm)
        self.max_gap_mm = float(max_gap_mm)
        self.min_scans = int(min_scans)

        self.scans = 0
        self.blades = 0          # clusters accepted, for the status line
        self._votes = {}         # bin index -> [count, sum of positions]
        self.bay = None          # (section, centre along the wall) once found
        self.bay_mm = NOMINAL_BAY_MM

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

        angles = np.radians(pose.heading + bearings[good])
        ranges = scan[good]
        x = pose.x + ranges * np.sin(angles)
        y = pose.y + ranges * np.cos(angles)

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
            return None

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
            return None
        # Centre to centre spans the gap plus one blade's thickness.
        self.bay_mm = max(0.0, best[2] - WALL_THICKNESS_MM)
        return self.section, best[1]

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
        return (f"bay: {self.bay[0]} at {self.bay[1] / 10:+.0f}cm, "
                f"{self.bay_mm / 10:.0f}cm wide")


# ============================================================================
# Driving into the bay
# ============================================================================
# Where the robot's own body sits relative to the point that tracks the path.
# The pose point is treated as the rear axle throughout the control stack
# (pursuit.rear_axle_offset_mm is 0), so these are measured from there.
BODY_FRONT_MM = 200.0       # rear axle to front bumper
BODY_REAR_MM = 40.0         # rear axle to rear bumper
BODY_HALF_WIDTH_MM = 60.0


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

    APPROACH, ARC_IN, STRAIGHT, ARC_OUT, SETTLE, DONE, ABORTED = (
        "approach", "arc_in", "straight", "arc_out", "settle", "done", "aborted")
    DRIVING = (ARC_IN, STRAIGHT, ARC_OUT, SETTLE)

    def __init__(self, frame, bay_mm=NOMINAL_BAY_MM,
                 turn_deg=45.0,
                 stage_depth_mm=350.0,
                 stage_along_offset_mm=None,
                 park_depth_mm=105.0,
                 end_offset_mm=0.0,
                 straight_mm=None,
                 turn_radius_mm=196.6,
                 line_depth_mm=600.0,
                 approach_mm=900.0,
                 approach_speed=25,
                 approach_lookahead_mm=200.0,
                 approach_lean_deg=25.0,
                 depth_gain=0.20,
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
        self.depth_gain = float(depth_gain)
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
        self._straight_done_mm = 0.0

        self.straight_mm, auto_offset = self.plan()
        self.stage_along_offset_mm = (auto_offset if stage_along_offset_mm is None
                                      else float(stage_along_offset_mm))
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
        """Where the rear axle starts, in bay coordinates."""
        return self.bay_mm / 2.0 + self.stage_along_offset_mm

    def summary(self):
        return (f"turn {self.turn_deg:.0f}deg  R {self.turn_radius_mm:.0f}mm  "
                f"stage {self.stage_depth_mm:.0f}mm out, axle "
                f"{self.stage_along_offset_mm:+.0f}mm past the far wall  "
                f"straight {self.straight_mm:.0f}mm  park {self.park_depth_mm:.0f}mm")

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

        return {self.ARC_IN: self._arc_in, self.STRAIGHT: self._straight,
                self.ARC_OUT: self._arc_out, self.SETTLE: self._settle}[self.phase](
                    dt, max_steer)

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
            return None                      # the racing line still has it
        if remaining <= 0.0:
            self._enter(self.ARC_IN)
            return (0.0, 0)

        # Lean toward the staging depth, then hold that lean.
        depth_error = self.d - self.stage_depth_mm       # + = still too far out
        wanted = clamp(-self.depth_gain * depth_error,
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
        self._straight_done_mm += abs(self.speed) / 100.0 * 700.0 * dt
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

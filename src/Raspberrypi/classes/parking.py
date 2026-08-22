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
    Backs the robot into the bay and squares it up.

    Phases, each closed-loop and each with a timeout:

        STAGE    still following the racing line - this returns no command, so
                 the path follower keeps driving - until the robot has gone
                 far enough PAST the bay to back into it
        SWING    reverse at full lock toward the wall until the nose has swung
                 `entry_deg` away from it
        PLUNGE   reverse straight, tail first, until the axle is at parking
                 depth
        SQUARE   alternate short forward and reverse strokes, both at the lock
                 that reduces yaw, until square. Short strokes and not one
                 closing arc: a single arc sweeps the nose corner
                 hypot(200, R + 60) from the final axle, which is 153mm at
                 R=60 and 207mm at R=146 against a blade face 150mm away - it
                 clips at BOTH plausible steering calibrations, so the design
                 must not use one
        SETTLE   nudge along the bay until centred, then stop

    Never touches the motor: update() returns (steer, speed) and the task
    applies it, so `steer_command`/`speed` stay the single record of what the
    wheels were told and the odometry keeps working through the manoeuvre.
    """

    STAGE, ENTRY, SQUARE, SETTLE, DONE, ABORTED = (
        "stage", "entry", "square", "settle", "done", "aborted")

    def __init__(self, frame, bay_mm=NOMINAL_BAY_MM, entry_deg=55.0,
                 park_depth_mm=90.0, stroke_mm=8.0, speed=45,
                 turn_radius_mm=196.6, line_depth_mm=600.0,
                 approach_mm=320.0, approach_speed=28, reach_mm=30.0,
                 square_deg=8.0, centre_tolerance_mm=12.0, depth_slack_mm=35.0,
                 wall_guard_mm=12.0, blade_guard_mm=10.0,
                 flip_penalty_mm2=900.0, work_deg=30.0, yaw_arm_mm=YAW_ARM_MM,
                 phase_timeout_s=20.0, max_strokes=60, stall_strokes=24):
        self.frame = frame
        self.bay_mm = float(bay_mm)
        self.entry_deg = float(entry_deg)
        self.park_depth_mm = float(park_depth_mm)
        self.stroke_mm = float(stroke_mm)
        self.turn_radius_mm = float(turn_radius_mm)
        self.line_depth_mm = float(line_depth_mm)
        self.approach_mm = float(approach_mm)
        self.approach_speed = int(approach_speed)
        # How far ahead the guard is asked to look when sizing a stroke.
        # Longer than a stroke, so the shuffle turns round before it runs out
        # of room rather than at the moment it does.
        self.reach_mm = float(reach_mm)
        self.flip_penalty_mm2 = float(flip_penalty_mm2)
        self.work_deg = float(work_deg)
        self.yaw_arm_mm = float(yaw_arm_mm)
        self.speed = int(speed)
        self.square_deg = float(square_deg)
        self.centre_tolerance_mm = float(centre_tolerance_mm)  # deadband on s
        self.depth_slack_mm = float(depth_slack_mm)
        self.wall_guard_mm = float(wall_guard_mm)
        self.blade_guard_mm = float(blade_guard_mm)
        self.phase_timeout_s = float(phase_timeout_s)
        self.max_strokes = int(max_strokes)
        self.stall_strokes = int(stall_strokes)

        self.phase = self.STAGE
        self.reason = None
        self.strokes = 0
        self.s = self.d = self.theta = 0.0
        self._elapsed = 0.0
        self._stroke_mm_done = 0.0
        self._reversing = True
        self._stroke = None          # the (direction, lock) being driven
        self._best_cost = float('inf')
        self._stale_strokes = 0

        # Where the axle has to finish for the body to sit centred, and how
        # far past the bay to drive before backing in - both straight out of
        # the geometry, see the module docstring.
        self.s_target = -(BODY_FRONT_MM - BODY_REAR_MM) / 2.0
        self.stage_s = self._stage_offset_mm(self.turn_radius_mm, self.line_depth_mm)

    # ------------------------------------------------------------------
    def _stage_offset_mm(self, radius_mm=60.0, line_depth_mm=474.0):
        """
        How far past the bay centre to stop before backing in.

        The swing arc and the straight plunge each eat some of the distance
        back to the bay; this is what is left over, so that the plunge ends
        with the axle at the bay centre rather than short of it or past it.
        """
        entry = math.radians(self.entry_deg)
        # The descent runs all the way to parking depth, so the only thing to
        # take off the top is what the swing itself already spent. The old
        # form reserved a second R*(1-cos(entry)) for an unwinding arc that no
        # longer exists; at the measured radius that was 70mm of depth the
        # descent then had to hunt for, and it spent the bay's whole length
        # doing it.
        plunge = ((line_depth_mm - self.park_depth_mm
                   - radius_mm * (1.0 - math.cos(entry)))
                  / max(0.1, math.sin(entry)))
        # Aim the descent at the axle position that centres the body AT THE
        # ENTRY ANGLE, not at the parked one. They differ by 80*(1-cos(entry))
        # - 34mm at 55 degrees - and using the parked value drops the robot
        # that far off centre, which is most of the room it has.
        #
        # There is deliberately no arc-lag term here any more. It belonged to
        # the single unwinding arc this used to end with, and biased the
        # landing 66mm further back at the measured radius - far enough that
        # the tail swung past the near blade and the shuffle could never
        # reverse again.
        landing = -(BODY_FRONT_MM - BODY_REAR_MM) / 2.0 * math.cos(entry)
        return radius_mm * math.sin(entry) + plunge * math.cos(entry) + landing

    # ------------------------------------------------------------------
    def update(self, pose, dt, max_steer=70.0):
        """
        One tick of the manoeuvre.

        I/O:
            pose: current Pose, in field coordinates
            return: (steer_command, speed), or None while the path follower
                    should stay in charge (STAGE) or the manoeuvre is over
        """
        if self.phase in (self.DONE, self.ABORTED):
            return None

        self.s, self.d, self.theta = self.frame.to_local(pose.x, pose.y, pose.heading)
        self._elapsed += dt

        if self.phase == self.STAGE:
            # Let the racing line carry the robot past the bay, then creep the
            # last stretch under our own power. At racing speed one tick is
            # 8mm, and every millimetre of overshoot here lands straight on
            # the tightest clearance in the whole manoeuvre - see
            # _arc_lag_mm - so the approach is deliberately slow.
            if self.s < self.stage_s - self.approach_mm:
                return None
            if self.s >= self.stage_s:
                self._enter(self.ENTRY)
                return (0.0, 0)
            # Hold it parallel to the wall on the way in; the heading is what
            # the swing arc is measured from.
            return (-self.frame.wall_side * clamp(2.0 * self.theta,
                                                  -max_steer, max_steer),
                    self.approach_speed)

        if self._elapsed > self.phase_timeout_s:
            return self._abort(f"{self.phase} timed out after {self._elapsed:.1f}s")
        breach = self._breach_mm()
        if breach is not None:
            return self._abort(breach)

        handler = {self.ENTRY: self._entry, self.SQUARE: self._square,
                   self.SETTLE: self._settle}[self.phase]
        return handler(dt, float(max_steer))

    # ------------------------------------------------------------------
    def _enter(self, phase):
        self.phase = phase
        self._elapsed = 0.0
        self._stroke_mm_done = 0.0
        self._stroke = None

    def _abort(self, reason):
        self.phase = self.ABORTED
        self.reason = reason
        print(f"Parking aborted: {reason}")
        return (0.0, 0)

    def _breach_mm(self):
        """The clearance guard, applied to where the robot actually is."""
        return self._breach_at(self.s, self.d, self.theta)

    def _breach_at(self, s, d, theta):
        """
        What, if anything, the chassis would go through at (s, d, theta).

        Kept free of `self`'s current pose so the shuffle can ask it about a
        stroke it has not driven yet. That is the whole point: with the bay
        barely bigger than the robot, the guard is not a last-resort abort but
        the thing that decides how long each stroke may be.
        """
        half = self.bay_mm / 2.0
        for cs, cd in self.frame.corners_local(s, d, theta):
            if cd < self.wall_guard_mm:
                return f"corner {self.wall_guard_mm - cd:.0f}mm inside the outer wall"
            # A blade only exists within the bay's depth; outside it the robot
            # is in open track and may overhang as far as it likes.
            if cd <= WALL_LENGTH_MM and abs(cs) > half - self.blade_guard_mm:
                return f"corner {abs(cs) - half + self.blade_guard_mm:.0f}mm into a bay wall"
        return None

    # ------------------------------------------------------------------
    # Looking one stroke ahead
    # ------------------------------------------------------------------
    def _here(self):
        return (self.s, self.d, self.theta)

    def _roll(self, state, direction, kappa, distance_mm, steps=6):
        """
        Where a stroke would leave the chassis, in bay-local coordinates.

        `kappa` is the yaw gained per millimetre TRAVELLED (not per millimetre
        of signed displacement), so the caller picks a yaw direction and this
        works out the wheel angle that delivers it - see _command_for. Doing it
        that way keeps every sign convention in one place instead of spread
        across the phases.
        """
        s, d, theta = state[0], state[1], math.radians(state[2])
        step = distance_mm / steps
        for _ in range(steps):
            s += direction * math.cos(theta) * step
            d += direction * math.sin(theta) * step
            theta += kappa * step
            yield s, d, math.degrees(theta)

    def _room_mm(self, state, direction, kappa, limit_mm=None):
        """How far this stroke can run from `state` before the guard stops it."""
        limit = self.reach_mm if limit_mm is None else limit_mm
        travelled = 0.0
        steps = max(2, int(limit / 4.0))
        for s, d, theta in self._roll(state, direction, kappa, limit, steps):
            if self._breach_at(s, d, theta) is not None:
                return travelled
            travelled += limit / steps
        return limit

    def _after(self, state, direction, kappa, distance_mm):
        """The state a stroke of `distance_mm` would end at."""
        end = state
        for end in self._roll(state, direction, kappa, distance_mm, steps=4):
            pass
        return end

    def _cost(self, state):
        """
        How far a state is from being parked, in millimetres-squared.

        The three axes are not equally tight and the weights say so: at the
        worst yaw the bay leaves about 16mm along its length but 40mm in
        depth, so `s` is worth roughly three times as much as `d`. Yaw is
        converted to a length by `yaw_arm_mm`, which is what makes the
        three commensurable at all - and it is the most delicate number
        here. Too large and the shuffle buys degrees at any price, walking
        itself out of the bay to square up in mid-air; too small and it
        sits centred and never turns.
        """
        s, d, theta = state
        along = s - self._centring_s_mm(theta)
        deep = d - self.park_depth_mm
        turned = math.radians(theta - self._yaw_goal_deg(d, theta)) * self.yaw_arm_mm
        return along * along + 0.35 * deep * deep + turned * turned

    def _yaw_goal_deg(self, d, theta):
        """
        The yaw the robot WANTS at this depth - which is not zero until the
        depth is right.

        A stroke moves the chassis in depth as sin(yaw), so a squared-up robot
        cannot descend at all. Costing yaw straight to zero therefore walks
        the shuffle into a corner: it spends the yaw first, arrives square
        100mm too far out, and then has nothing left to steer with. Two
        strokes of lookahead cannot see round that, because rebuilding the yaw
        looks worse before it looks better.

        Holding a working angle until the depth is close removes the trap
        without needing a deeper search. The sign follows whichever way the
        robot is already leaning, so this never asks it to swing through
        square and back.

        It fades in with the depth error rather than switching at a
        threshold. A step here puts a cliff in the cost, and the search sat on
        it and chattered: one side of the line wanted to square up, the other
        wanted the yaw back, and the robot alternated every tick without
        moving at all.
        """
        short = abs(d - self.park_depth_mm) / max(1.0, self.depth_slack_mm)
        return math.copysign(self.work_deg * min(1.0, short),
                             theta if theta else 1.0)

    def _command_for(self, direction, kappa, max_steer):
        """
        The steering command that produces `kappa` while travelling in
        `direction`. Reversing flips which way the wheels must point, and so
        does having the wall on the other side, which is why this is derived
        rather than written out per phase.
        """
        if not kappa:
            return 0.0
        return (-math.copysign(1.0, kappa) * self.frame.wall_side
                * direction * max_steer)

    def _entry(self, dt, max_steer):
        """
        Back in: swing to the entry angle, then reverse straight down it.

        This was one closed-loop descent when the robot turned inside 60mm,
        because at that radius a control tick was 6 degrees of swing and there
        was no entry angle precise enough to aim with. At the measured lock the
        radius is 197mm, a tick is 1.7 degrees, and the arithmetic reverses:
        the descent can no longer be steered onto a landing point at all
        (unwinding from 197mm sweeps the nose straight through a blade,
        whatever the gain), while the swing is now stoppable.

        So the aim moved. Entry no longer tries to land centred - it only has
        to get the tail into the bay without touching anything. Getting
        CENTRED is the shuffle's job, and the shuffle can translate as well as
        rotate, which a single arc cannot.
        """
        if self.d <= self._depth_target_mm():
            self._enter(self.SQUARE)
            self._reversing = True
            return (0.0, 0)

        building = self.theta < self.entry_deg
        kappa = (1.0 / self.turn_radius_mm) if building else 0.0
        need = 2.0 * abs(self.speed) / 100.0 * 700.0 * dt + 2.0
        if self._room_mm(self._here(), -1.0, kappa) < need:
            # Far enough in that the next bite would clip: hand over early
            # rather than abort. The shuffle starts from wherever this got to.
            self._enter(self.SQUARE)
            self._reversing = True
            return (0.0, 0)
        return (self._command_for(-1.0, kappa, max_steer), -self.speed)

    def _square(self, dt, max_steer):
        """
        Unwind the yaw with short strokes, each one run until the guard says
        stop rather than to a fixed length.

        Both directions of travel reduce the yaw, as long as the lock is set
        to the one that unwinds it - so the direction is free to be used for
        something else. Two things want it, and they are ranked:

          * whether there is ROOM to go that way at all. At 197mm of turning
            radius the bay holds about 8mm of clearance at its tightest, and a
            fixed stroke length either wastes most of the room or drives
            through a blade. Asking the guard how far it can go turns that
            8mm into the control law instead of a tripwire.
          * where the robot sits ALONG the bay, which is the tight axis: the
            bay is 200mm deep against a 120mm-wide robot, so depth has about
            +/-40mm of room, while at the worst yaw `s` has 16mm.

        Room wins, because a stroke that cannot be driven is not a choice.
        """
        if (abs(self.theta) <= self.square_deg
                and abs(self.d - self.park_depth_mm) <= self.depth_slack_mm):
            self._enter(self.SETTLE)
            return (0.0, 0)
        if self.strokes >= self.max_strokes:
            return self._abort(f"still {self.theta:.0f}deg off square after "
                               f"{self.strokes} strokes")
        # A limit cycle looks exactly like work from the inside - strokes tick
        # up, the wheels move - so progress is measured against the cost
        # rather than against the stroke count.
        cost = self._cost(self._here())
        if cost < self._best_cost - 25.0:
            self._best_cost, self._stale_strokes = cost, self.strokes
        elif self.strokes - self._stale_strokes >= self.stall_strokes:
            return self._abort(f"shuffle stopped gaining {self.theta:.0f}deg off "
                               f"square, {self.d - self.park_depth_mm:+.0f}mm of depth")

        # Two ticks plus a little, because a stroke is committed for a whole
        # tick before this runs again and the servo does not move instantly.
        # Checking room only when the stroke ENDS is what put a corner 3mm
        # into a blade: room was 8mm, the stroke ran 14.
        tick = abs(self.speed) / 100.0 * 700.0 * dt
        need = 2.0 * tick + 2.0
        self._stroke_mm_done += tick
        cornered = (self._stroke is not None
                    and self._room_mm(self._here(), *self._stroke) < need)
        if self._stroke is None or self._stroke_mm_done >= self.stroke_mm or cornered:
            # When the guard is what ended the stroke, ask the replacement
            # for a little more than the bare minimum. Accepting anything
            # merely legal let the two directions take turns being blocked,
            # flipping every tick and travelling nothing: 130 strokes for 0mm.
            # Only a little, though - around 27 degrees the bay is down to 8mm
            # of wiggle per end, and that is exactly where the shuffle needs
            # its shortest strokes.
            choice = self._choose_stroke(need * 1.5 if cornered else need)
            if choice is None:
                return self._abort(f"shuffle stuck {self.theta:.0f}deg off square, "
                                   f"{self.d - self.park_depth_mm:+.0f}mm of depth to go")
            if self._stroke is not None and choice[0] != self._stroke[0]:
                # A stroke is a CHANGE OF DIRECTION, not a change of plan.
                # The room check above re-plans most ticks, so counting plans
                # burned the whole stroke budget before the robot had moved.
                self.strokes += 1
            if choice != self._stroke:
                self._stroke_mm_done = 0.0
            self._stroke = choice
        direction, kappa = self._stroke
        self._reversing = direction < 0.0
        return (self._command_for(direction, kappa, max_steer),
                -self.speed if self._reversing else self.speed)

    def _choose_stroke(self, need_mm):
        """
        The (direction, lock) that gets closest to parked, two strokes out.

        Depth and yaw cannot be handled one at a time, which is what the
        earlier fixed-lock shuffle got wrong: it always chose the lock that
        unwound the yaw, so the yaw was spent long before the depth was, and
        the robot squared up 100mm too far out with no yaw left to descend on
        (a stroke moves the chassis in depth as sin(yaw), so at 8 degrees off
        square there is almost nothing left to steer with). Choosing the lock
        as well lets a stroke trade one against the other.

        Two strokes rather than one because the shuffle is a pair of moves by
        nature - a stroke that helps on its own is often the one that leaves
        nowhere to go next. Six options each way is 36 rollouts, a few hundred
        microseconds, and it removes the whole class of stall.
        """
        locks = (0.0, 1.0 / self.turn_radius_mm, -1.0 / self.turn_radius_mm)
        here = -1.0 if self._reversing else 1.0
        best = None
        for direction in (1.0, -1.0):
            for kappa in locks:
                room = self._room_mm(self._here(), direction, kappa)
                if room < need_mm:
                    continue
                first = self._after(self._here(), direction, kappa,
                                    min(room, self.reach_mm))
                horizon = None
                for way in (1.0, -1.0):
                    for lock in locks:
                        room2 = self._room_mm(first, way, lock)
                        if room2 < need_mm:
                            continue
                        end = self._after(first, way, lock, min(room2, self.reach_mm))
                        cost = self._cost(end)
                        horizon = cost if horizon is None else min(horizon, cost)
                if horizon is None:
                    horizon = self._cost(first)
                # Changing direction costs a stop, a servo sweep and the
                # backlash in between, so it has to be worth something.
                score = horizon + (self.flip_penalty_mm2 if direction != here else 0.0)
                if best is None or score < best[0]:
                    best = (score, (direction, kappa))
        return None if best is None else best[1]


    def _s_target_mm(self):
        """
        Where the axle should be RIGHT NOW, for the body's middle to sit over
        the middle of the bay at the current yaw.

        Not a constant. The axle is not in the middle of the robot - it is
        80mm behind it - so as the robot turns, the axle position that centres
        the body swings by 80*cos(yaw). Aiming at the parked value throughout
        leaves the body offset by up to that much exactly while it is turning
        through its widest footprint, which is where the bay has only 16mm to
        spare per end. Tracking the yaw instead keeps the clearance balanced
        at every angle.
        """
        return self._centring_s_mm(self.theta)

    @staticmethod
    def _centring_s_mm(theta_deg):
        """The axle position that centres the body, at a yaw of theta."""
        return (-(BODY_FRONT_MM - BODY_REAR_MM) / 2.0
                * math.cos(math.radians(theta_deg)))

    def _depth_target_mm(self):
        """
        How deep the axle should be: all the way, while the yaw is still high.

        This used to stop the descent short by R*(1-cos t), to leave room for
        an unwinding arc to spend. There is no such arc any more, and at the
        measured radius that reservation was 100mm - which stranded the robot
        at exactly the yaw where the bay is tightest.

        The robot's footprint ALONG the bay is 240*cos(t) + 120*sin(t), which
        peaks at 268mm around 27 degrees and falls to 246mm by 50. In a 300mm
        bay that is the difference between 8mm of wiggle per end and 27mm. So
        the depth has to be won while the yaw is still high, and the tight
        angles crossed afterwards by rotating on the spot rather than by
        trying to translate through them.
        """
        return self.park_depth_mm


    def _settle(self, dt, max_steer):
        error = self.s - self._s_target_mm()
        if abs(error) <= self.centre_tolerance_mm:
            self.phase = self.DONE
            return (0.0, 0)
        # Square already, so this is a straight nudge along the bay.
        return (0.0, -self.speed if error > 0.0 else self.speed)

    # ------------------------------------------------------------------
    @property
    def active(self):
        return self.phase not in (self.STAGE, self.DONE, self.ABORTED)

    @property
    def finished(self):
        return self.phase in (self.DONE, self.ABORTED)

    def status_line(self):
        return (f"park {self.phase} s={self.s:+.0f} d={self.d:.0f} "
                f"yaw={self.theta:+.0f}deg strokes={self.strokes}")

"""
Walls, as the lidar actually sees them.

Everything here answers one question: WHERE IS THE WALL, RIGHT NOW. Not where
the localizer thinks the robot is - where the wall is, measured this tick,
from this scan. That distinction is the whole point of the module.

WHY A FITTED LINE AND NOT A RAY
-------------------------------
`LidarManager.get_min_distance()` returns the CLOSEST return in a sector, and
for holding a distance from a wall that is the wrong number twice over:

    A pillar standing near the wall is closer than the wall. The sector query
    returns the pillar, the controller reads it as "the wall just jumped 300mm
    nearer", and the robot swerves away from a wall that never moved. On the
    obstacle round there is a pillar beside the wall for most of the lap.

    A single ray is not the perpendicular. Yawed by psi, a beam fired at 90
    degrees measures perpendicular/cos(psi) - 6% long at 20 degrees, 15% at
    30. The robot is yawed exactly when it is correcting, so the error arrives
    precisely when the controller is trying to settle, and it reads as
    overshoot that is not there.

A line fitted through the whole arc has neither problem. The pillar is a
handful of outliers that do not fit and get thrown out; the perpendicular
distance to a fitted line does not care how the robot is pointed.

The fitting core (`fit_line`, `without_outliers`, `wall_yaw`) is lifted from
ParkingSequence, where it has been measuring yaw to about two degrees against
the eight a two-ray difference managed. `parking.py` now imports it from here
so there is one implementation and not two.

FRAME
-----
Robot frame throughout, matching the rest of the project: x is to the right,
y is straight ahead, and a bearing is degrees CLOCKWISE from the nose - so a
point at bearing b and range d is at (d*sin b, d*cos b), the same convention
`FieldMap` uses for headings and `BlockMap._to_field` uses for detections.
"""
import math

import numpy as np


# ============================================================================
# Fitting a wall to an arc of scan  (extracted from ParkingSequence)
# ============================================================================
# Below this the "spread" of a candidate arc is too small to give a baseline:
# all the points are bunched at one bearing and the slope is noise over noise.
MIN_FIT_SPREAD = 100.0
# An outlier is 3 sigma out, but never less than this - with fifty points on a
# flat wall the RMS collapses to a millimetre or two and a pure 3-sigma rule
# would start rejecting the wall itself.
MIN_OUTLIER_LIMIT_MM = 40.0


def fit_line(points):
    """
    Least squares depth = intercept + slope * along, or None.

    With the wall on the right and the robot yawed by psi, every return
    satisfies depth = D/cos(psi) + along*tan(psi) exactly - so the slope IS
    the tangent of the yaw, and the depth cancels out of it.
    """
    count = len(points)
    if count < 2:
        return None
    mean_along = sum(p[0] for p in points) / count
    mean_depth = sum(p[1] for p in points) / count
    spread = sum((p[0] - mean_along) ** 2 for p in points)
    if spread < MIN_FIT_SPREAD:     # all bunched up: no baseline, no angle
        return None
    cross = sum((p[0] - mean_along) * (p[1] - mean_depth) for p in points)
    slope = cross / spread
    return slope, mean_depth - slope * mean_along


def without_outliers(points, line):
    """
    The points that the fit actually explains.

    One pass is enough and more than one is harmful: this is here to drop a
    pillar or a bay blade that has wandered into the arc, not to polish the
    fit until only the points that agree with it survive.
    """
    slope, intercept = line
    residuals = [p[1] - (slope * p[0] + intercept) for p in points]
    rms = math.sqrt(sum(r * r for r in residuals) / len(residuals))
    limit = max(3.0 * rms, MIN_OUTLIER_LIMIT_MM)
    return [p for p, r in zip(points, residuals) if abs(r) <= limit]


def wall_yaw(points, min_points, max_deg):
    """
    Yaw relative to the wall the points lie on, in degrees.

    I/O:
        return: (yaw_deg, fit_was_clean). yaw is nan when the arc does not
                look like a wall - too few points, no baseline, or a slope
                steeper than `max_deg`, which means a corner or a step rather
                than the wall we meant.

    WHETHER ANYTHING WAS THROWN OUT IS ITSELF A MEASUREMENT. A clean fit means
    the arc saw one flat thing; a dirty one means something else has entered
    it, and callers use that as an early warning several ticks before the
    range itself moves.
    """
    if len(points) < min_points:
        return float("nan"), False
    line = fit_line(points)
    if line is None:
        return float("nan"), False
    kept = without_outliers(points, line)
    if len(kept) < min_points:
        return float("nan"), False
    clean = len(kept) == len(points)
    if not clean:
        line = fit_line(kept)
        if line is None:
            return float("nan"), False
    yaw = math.degrees(math.atan(line[0]))
    if abs(yaw) > max_deg:
        return float("nan"), clean      # a corner or a step, not the wall
    return yaw, clean


def arc_points(scan, centre_bearing_deg, half_arc_deg, side,
               min_range_mm, max_range_mm, fov_limit_deg):
    """
    An arc of the scan as (along, depth) millimetres, ready for `fit_line`.

    Mirrored by `side` into the frame where the wall is on the right and the
    nose points along +along, so everything downstream is written once and the
    wall side only appears when the scan is indexed.

    THE ARC STOPS SHORT OF THE BLIND SPOT. Past `fov_limit_deg` the scan
    returns the robot's own body at ~110mm, which reads as a wall crashing in
    at 45 degrees. The arc is clamped there rather than trusted to a config.
    """
    if scan is None or len(scan) < 360:
        return []
    first = int(round(centre_bearing_deg - half_arc_deg))
    last = int(round(min(centre_bearing_deg + half_arc_deg, fov_limit_deg)))
    points = []
    for bearing in range(first, last + 1):
        distance = float(scan[int(round(bearing * side)) % 360])
        if math.isnan(distance):
            continue
        if not min_range_mm < distance < max_range_mm:
            continue
        radians = math.radians(bearing)
        points.append((distance * math.cos(radians),
                       distance * math.sin(radians)))
    return points


# ============================================================================
# A wall as a line segment
# ============================================================================
class LineSegment:
    """
    Two endpoints in the robot frame, plus the two questions worth asking of
    a wall: how far away is it, and which way is it.

    Distances are to the INFINITE line, not the segment. That is deliberate:
    the visible end of a wall is wherever the scan happened to stop, and
    clamping to it would make the measured distance jump every time a pillar
    occluded the far end.
    """

    __slots__ = ("x1", "y1", "x2", "y2")

    def __init__(self, x1, y1, x2, y2):
        self.x1, self.y1, self.x2, self.y2 = x1, y1, x2, y2

    def length(self):
        return math.hypot(self.x2 - self.x1, self.y2 - self.y1)

    def perpendicular_distance(self, x=0.0, y=0.0):
        """Distance from a point to the infinite line, in millimetres."""
        dx = self.x2 - self.x1
        dy = self.y2 - self.y1
        den = math.hypot(dx, dy)
        if den < 1e-6:
            return 0.0
        return abs(dy * x - dx * y + self.x2 * self.y1 - self.y2 * self.x1) / den

    def perpendicular_bearing(self, x=0.0, y=0.0):
        """
        Bearing from a point TOWARD the line, degrees clockwise from the nose.

        This is what tells front from left from right, so it has to point at
        the wall rather than merely along its normal - hence the sign flip.
        """
        dx = self.x2 - self.x1
        dy = self.y2 - self.y1
        den = math.hypot(dx, dy)
        if den < 1e-6:
            return 0.0
        # Normal to the segment, either sense.
        nx, ny = -dy / den, dx / den
        # Signed distance from the query point to the line along that normal.
        offset = (self.x1 - x) * nx + (self.y1 - y) * ny
        if offset < 0.0:
            nx, ny = -nx, -ny
        return math.degrees(math.atan2(nx, ny)) % 360.0

    def angle_deg(self):
        """Orientation of the segment itself, 0-180, for collinearity tests."""
        return math.degrees(math.atan2(self.x2 - self.x1,
                                       self.y2 - self.y1)) % 180.0

    def foot_overhang(self, x=0.0, y=0.0):
        """
        How far PAST an end of the segment the perpendicular foot falls, mm.

        Zero when the foot lands on the segment itself. Distances everywhere
        else in this class are to the infinite line, on purpose - but that is
        exactly what lets a wall the robot will never reach answer "how far
        until I hit something". This is the question that tells them apart.
        """
        dx = self.x2 - self.x1
        dy = self.y2 - self.y1
        den = dx * dx + dy * dy
        if den < 1e-9:
            return math.hypot(x - self.x1, y - self.y1)
        # Position of the foot along the segment, 0 at one end, 1 at the other.
        t = ((x - self.x1) * dx + (y - self.y1) * dy) / den
        if 0.0 <= t <= 1.0:
            return 0.0
        return (t - 1.0 if t > 1.0 else -t) * math.sqrt(den)

    def __repr__(self):
        return (f"LineSegment(({self.x1:.0f},{self.y1:.0f})-"
                f"({self.x2:.0f},{self.y2:.0f}) {self.length():.0f}mm)")


# ============================================================================
# Scan -> line segments
# ============================================================================
# The field is 3m corner to corner on the diagonal; nothing further is ours.
MAX_POINT_RANGE_MM = 3200.0
MIN_POINT_RANGE_MM = 5.0
# Long returns are only trusted ahead. Behind the robot a distant return is
# almost always a spectator, a ceiling light or a beam that slipped past the
# centre block - and one of those fitted as a wall is a phantom corner.
REAR_TRUST_RANGE_MM = 700.0
REAR_ARC_START_DEG = 100.0
REAR_ARC_END_DEG = 260.0

# THE ROBOT SEES ITSELF. Behind the mast the beam hits our own chassis and
# comes back at about 110mm, all the way round the rear arc. `arc_points`
# has always known this - it clamps the parking arc short of the blind spot
# for exactly this reason - and NavigationManager.crop_scan blanks the same
# wedge so "the robot's own body can never be mistaken for a wall". This
# path had neither guard, and the rear mask above made it worse rather than
# better: it keeps CLOSE returns behind us, which is precisely what the
# chassis produces.
#
# Left in, the arc fits as two ~100mm segments about 95mm out. One lands in
# `left` or `right`, where `nearest` prefers it over a wall half a metre
# away, and the lateral ruler reads 95mm in a lane it should be holding at
# 500 - so the cascade hauls the robot off its line and into the centre
# block. The other lands in `back`, where 3000 - 98 becomes a front-wall
# reading of 2902mm and the corner trigger loses its ruler. One artifact,
# both of the round's failure modes.
BODY_ARC_START_DEG = 120.0     # matches navigation_manager.DEFAULT_FOV_DEG
BODY_ARC_END_DEG = 240.0
BODY_RADIUS_MM = 150.0         # chassis is ~110mm; the margin is scan noise

# Split where the run bows more than this away from its chord.
SPLIT_DEVIATION_MM = 50.0
# Split where consecutive returns jump - that is an edge, not a bend.
SPLIT_GAP_MM = 100.0
MIN_SEGMENT_POINTS = 10
MIN_SEGMENT_MM = 100.0

# A WALL IS NOT A SHORT LINE. 100mm is the right floor for "is this a line at
# all" during the split, but a segment that survives merging and is still only
# a hand's width long is a pillar, and offering it to classify() is how a
# pillar becomes "the nearest thing on my left".
#
# Measured, driving a lap: a genuine wall fits at 1000-1650mm and reads within
# a few mm of truth. The bad ticks - reading 70 to 95mm NEARER than the wall
# actually is - fit at 100-235mm every time, and always nearer, because a
# pillar stands between the robot and the wall it is hiding. That error is
# too small for the lap controller's 120mm continuity gate to catch and big
# enough that the wall PID turns it into 30 degrees of steering, which is
# most of the weave on a straight.
#
# 300mm is the ported value (lidar_processor getRelativeWalls minLength) and
# it sits in the gap: 65mm above the largest pillar artifact seen, 700mm
# below the shortest real wall. Applied AFTER merging, unlike theirs, so a
# wall a pillar cut in half is restitched first and then measured whole.
MIN_WALL_MM = 300.0

# Two segments are the same wall if they point the same way and meet.
MERGE_ANGLE_DEG = 18.0
MERGE_ENDPOINT_MM = 200.0
# ...or, for segments that are not neighbours in the sweep, if one lies along
# the other's infinite line. This is what restitches a wall a pillar cut in
# half - the two halves are far apart in the scan but perfectly collinear.
ALIGN_OFFSET_MM = 220.0
ALIGN_SAMPLES = 10


def scan_points(scan):
    """
    The scan as (x, y) millimetres in the robot frame, in sweep order.

    Sweep order matters: everything downstream treats adjacency in this list
    as angular adjacency, which is what makes a simple split-and-merge work
    without any spatial indexing.
    """
    if scan is None or len(scan) < 360:
        return []
    bearings = np.arange(360, dtype=float)
    ranges = np.asarray(scan, dtype=float)

    good = ~np.isnan(ranges)
    good &= (ranges > MIN_POINT_RANGE_MM) & (ranges < MAX_POINT_RANGE_MM)
    # The range-dependent rear mask: behind us, only believe close returns.
    rear = (bearings >= REAR_ARC_START_DEG) & (bearings <= REAR_ARC_END_DEG)
    good &= ~(rear & (ranges > REAR_TRUST_RANGE_MM))
    # ...but not THAT close. Inside the body arc a short return is us.
    body = (bearings >= BODY_ARC_START_DEG) & (bearings <= BODY_ARC_END_DEG)
    good &= ~(body & (ranges < BODY_RADIUS_MM))

    keep = np.flatnonzero(good)
    if keep.size == 0:
        return []
    radians = np.radians(bearings[keep])
    xs = ranges[keep] * np.sin(radians)
    ys = ranges[keep] * np.cos(radians)
    return list(zip(xs.tolist(), ys.tolist()))


def _split_runs(points):
    """Break the sweep wherever consecutive returns jump more than a gap."""
    runs = []
    current = [points[0]]
    for previous, point in zip(points, points[1:]):
        if math.hypot(point[0] - previous[0], point[1] - previous[1]) > SPLIT_GAP_MM:
            runs.append(current)
            current = [point]
        else:
            current.append(point)
    runs.append(current)
    return runs


def _split_bends(run):
    """
    Ramer-Douglas-Peucker: recursively split a run at its worst bow.

    Iterative rather than recursive because a 360-point scan on a bad tick can
    nest deeper than is comfortable, and this runs inside a 50Hz loop.
    """
    out = []
    stack = [(0, len(run) - 1)]
    while stack:
        start, end = stack.pop()
        if end - start + 1 < MIN_SEGMENT_POINTS:
            continue
        ax, ay = run[start]
        bx, by = run[end]
        chord = LineSegment(ax, ay, bx, by)
        if chord.length() < 1e-6:
            continue
        worst, worst_at = 0.0, -1
        for index in range(start + 1, end):
            deviation = chord.perpendicular_distance(*run[index])
            if deviation > worst:
                worst, worst_at = deviation, index
        if worst > SPLIT_DEVIATION_MM and worst_at > 0:
            stack.append((start, worst_at))
            stack.append((worst_at, end))
        else:
            out.append((start, end))
    return out


def _segment_from(run, start, end):
    """Fit the run's own extent rather than trusting its two endpoints."""
    points = run[start:end + 1]
    if len(points) < MIN_SEGMENT_POINTS:
        return None
    segment = LineSegment(points[0][0], points[0][1], points[-1][0], points[-1][1])
    if segment.length() < MIN_SEGMENT_MM:
        return None
    return segment


def _collinear(first, second):
    difference = abs(first.angle_deg() - second.angle_deg()) % 180.0
    return min(difference, 180.0 - difference) <= MERGE_ANGLE_DEG


def _joined(first, second):
    """The merged segment spanning both, along the dominant direction."""
    corners = [(first.x1, first.y1), (first.x2, first.y2),
               (second.x1, second.y1), (second.x2, second.y2)]
    dx = first.x2 - first.x1
    dy = first.y2 - first.y1
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return first
    dx, dy = dx / norm, dy / norm
    projections = [(corner[0] * dx + corner[1] * dy, corner) for corner in corners]
    projections.sort(key=lambda item: item[0])
    (_, low), (_, high) = projections[0], projections[-1]
    return LineSegment(low[0], low[1], high[0], high[1])


def _endpoints_meet(first, second):
    pairs = (((first.x2, first.y2), (second.x1, second.y1)),
             ((first.x1, first.y1), (second.x2, second.y2)),
             ((first.x2, first.y2), (second.x2, second.y2)),
             ((first.x1, first.y1), (second.x1, second.y1)))
    return any(math.hypot(a[0] - b[0], a[1] - b[1]) <= MERGE_ENDPOINT_MM
               for a, b in pairs)


def merge_sequential(segments):
    """
    Merge neighbours in the sweep that point the same way and meet.

    INCLUDING FIRST AGAINST LAST. The scan wraps at 0/360, so a wall lying
    across the seam arrives as two half-walls at opposite ends of the list.
    Without the wrap-around merge the robot sees two short walls where there
    is one long one, and `resolve` picks the nearer half.
    """
    if len(segments) < 2:
        return list(segments)
    merged = [segments[0]]
    for segment in segments[1:]:
        if _collinear(merged[-1], segment) and _endpoints_meet(merged[-1], segment):
            merged[-1] = _joined(merged[-1], segment)
        else:
            merged.append(segment)
    if (len(merged) > 1 and _collinear(merged[-1], merged[0])
            and _endpoints_meet(merged[-1], merged[0])):
        merged[0] = _joined(merged.pop(), merged[0])
    return merged


def merge_aligned(segments):
    """
    Merge segments that are not neighbours but lie on the same line.

    A pillar standing in front of a wall cuts it into two pieces with a gap
    between them; so does the centre block seen past a corner. Sequential
    merging cannot rejoin those because the pieces are not adjacent in the
    sweep. Run to a fixpoint - rejoining two halves can make a third piece
    collinear with the result.
    """
    working = list(segments)
    changed = True
    while changed and len(working) > 1:
        changed = False
        for i in range(len(working)):
            for j in range(i + 1, len(working)):
                first, second = working[i], working[j]
                if not _collinear(first, second):
                    continue
                # Every sampled point of one must lie along the other's line.
                far = max(first.perpendicular_distance(second.x1, second.y1),
                          first.perpendicular_distance(second.x2, second.y2))
                if far > ALIGN_OFFSET_MM:
                    continue
                if not _sampled_along(first, second):
                    continue
                working[i] = _joined(first, second)
                working.pop(j)
                changed = True
                break
            if changed:
                break
    return working


def _sampled_along(first, second):
    """Check the interpolated span, not just the endpoints."""
    for step in range(ALIGN_SAMPLES + 1):
        fraction = step / float(ALIGN_SAMPLES)
        x = second.x1 + (second.x2 - second.x1) * fraction
        y = second.y1 + (second.y2 - second.y1) * fraction
        if first.perpendicular_distance(x, y) > ALIGN_OFFSET_MM:
            return False
    return True


def get_lines(scan):
    """Scan in, list of wall segments out. The whole extraction pipeline."""
    points = scan_points(scan)
    if len(points) < MIN_SEGMENT_POINTS:
        return []
    segments = []
    for run in _split_runs(points):
        if len(run) < MIN_SEGMENT_POINTS:
            continue
        for start, end in _split_bends(run):
            segment = _segment_from(run, start, end)
            if segment is not None:
                segments.append(segment)
    return merge_aligned(merge_sequential(segments))


# ============================================================================
# Segments -> named walls
# ============================================================================
# Nothing beyond this can be the wall of the lane we are in: the lane is a
# metre wide and the field only three, so a "side wall" at 1.5m is the far
# side of the field pretending.
MAX_SIDE_WALL_MM = 1200.0
# A front or back ruler has to lie ACROSS the robot's own track. The centre
# block's face is perpendicular to travel, has its normal pointing at us and
# is about a metre long, so nothing about its shape distinguishes it from the
# wall at the end of the straight - and once a pillar shadows the sliver of
# real wall the block leaves visible, it is the only front candidate left.
#
# What does distinguish it is where it sits ACROSS the lane. The block is
# 1000mm square in a 3000mm field, so its face starts 1000mm in from the outer
# wall - past the far edge of the 1000mm lane. Even in the innermost lane we
# ever command (760mm) the near corner of the block is 240mm off our
# centreline, so a foot that overhangs by more than this is not our wall.
# Sides are exempt: the block's side faces ARE the inner lane wall.
MAX_FRONT_OVERHANG_MM = 150.0
# The band a backup reference is allowed to live in, when the near wall is
# occluded by a pillar for a few ticks.
FAR_WALL_MIN_MM = 1200.0
FAR_WALL_MAX_MM = 3200.0


class ResolvedWalls:
    """One wall per side, or None. The controller's entire view of the world."""

    __slots__ = ("front", "back", "left", "right", "far_left", "far_right")

    def __init__(self, front=None, back=None, left=None, right=None,
                 far_left=None, far_right=None):
        self.front, self.back = front, back
        self.left, self.right = left, right
        self.far_left, self.far_right = far_left, far_right

    def distance(self, name, default=float("nan")):
        segment = getattr(self, name, None)
        if segment is None:
            return default
        return segment.perpendicular_distance(0.0, 0.0)

    def __repr__(self):
        parts = [f"{name}={self.distance(name):.0f}"
                 for name in ("front", "back", "left", "right")
                 if getattr(self, name) is not None]
        return f"ResolvedWalls({', '.join(parts) or 'nothing'})"


def classify(segments, heading_deg, target_heading_deg):
    """
    Bin segments into front/back/left/right - IN THE TARGET FRAME.

    This is the one line that makes the whole controller work, so it is worth
    being explicit about why it is not the robot frame.

    Mid-correction the robot crabs. Bin by where a wall sits relative to the
    NOSE and a 20-degree crab starts calling the front wall a side wall, the
    front-wall ruler jumps, and the corner trigger fires in the wrong place.
    Bin by where it sits relative to the direction we are TRYING to go and the
    answer does not move at all while the robot swings back onto line.

    A wall at robot-frame bearing b sits at field bearing (heading + b), so in
    the target frame it sits at b + (heading - target). Note this ADDS the
    heading error where the C++ original subtracts it: their perpendicular
    direction is measured anticlockwise from +x, ours is a bearing clockwise
    from the nose, and the two conventions differ by a sign.
    """
    error = heading_deg - target_heading_deg
    walls = {"front": [], "right": [], "back": [], "left": []}
    for segment in segments:
        if segment.length() < MIN_WALL_MM:
            continue                      # a pillar, or a scrap of one
        bearing = (segment.perpendicular_bearing(0.0, 0.0) + error) % 360.0
        if bearing >= 315.0 or bearing < 45.0:
            side = "front"
        elif bearing < 135.0:
            side = "right"
        elif bearing < 225.0:
            side = "back"
        else:
            side = "left"
        if (side in ("front", "back")
                and segment.foot_overhang(0.0, 0.0) > MAX_FRONT_OVERHANG_MM):
            continue                      # beside the lane, not across it
        walls[side].append(segment)
    return walls


def resolve(walls):
    """
    Pick one segment per side.

    SIDES TAKE THE NEAREST, FRONT AND BACK TAKE THE FARTHEST. The asymmetry is
    not an oversight. Beside the robot the nearest flat thing within a metre
    IS the lane wall. Ahead of it the nearest thing is very often a pillar
    whose face fitted as a short segment, and steering the corner trigger off
    a pillar would fire it two metres early - so the front ruler deliberately
    reaches past clutter to the boundary behind it.
    """
    def nearest(candidates, limit):
        usable = [segment for segment in candidates
                  if segment.perpendicular_distance(0.0, 0.0) <= limit]
        if not usable:
            return None
        return min(usable, key=lambda s: s.perpendicular_distance(0.0, 0.0))

    def farthest(candidates):
        if not candidates:
            return None
        return max(candidates, key=lambda s: s.perpendicular_distance(0.0, 0.0))

    def in_band(candidates):
        usable = [segment for segment in candidates
                  if FAR_WALL_MIN_MM
                  <= segment.perpendicular_distance(0.0, 0.0)
                  <= FAR_WALL_MAX_MM]
        if not usable:
            return None
        return min(usable, key=lambda s: s.perpendicular_distance(0.0, 0.0))

    return ResolvedWalls(
        front=farthest(walls["front"]),
        back=farthest(walls["back"]),
        left=nearest(walls["left"], MAX_SIDE_WALL_MM),
        right=nearest(walls["right"], MAX_SIDE_WALL_MM),
        far_left=in_band(walls["left"]),
        far_right=in_band(walls["right"]),
    )


def resolve_walls(scan, heading_deg, target_heading_deg):
    """Scan in, one wall per side out. The whole module in one call."""
    return resolve(classify(get_lines(scan), heading_deg, target_heading_deg))

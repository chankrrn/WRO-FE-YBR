"""
How far away is that pillar, really?

The camera is excellent at two of the three things we need. Colour is a
threshold on a saturated hue and it is not close to failing. Bearing is
pixel position, which is as accurate as the lens calibration. RANGE is the
weak one: ObjectSolver derives it from apparent box height, so

    distance = (BOX_HEIGHT/2) / tan(height_px * DEG_PER_PX / 2)

and the error grows as 1/height_px. A box clipped by the frame edge, or
half-hidden behind another pillar, reads far too distant - the measurement
degrades in the exact situations where being wrong is most expensive.

Slot classification needs about +-150mm to land inside the dead-bands. So
this module keeps the camera's colour and bearing untouched and replaces
only the range, by asking the lidar what is actually along that bearing.

NOTHING IN THE VISION PIPELINE CHANGES. This consumes ObjectSolver's output
and hands back a corrected range; if the lidar has nothing to say, the
camera's own number is passed through, tagged, and the caller decides.

    Why bearings are matched rather than rays cast

The camera sits 170mm behind the lidar on the same mast, so the two sensors
disagree about the bearing to a nearby pillar - by 10 degrees at 1m, which
is more than the association window. Rather than casting a ray from the
camera and intersecting it with lidar geometry, every lidar point is
projected INTO the camera frame and its bearing compared there. Same
fusion, no intersection maths, and the baseline is handled exactly.

    Why clusters here are tiny

lidar_manager bins returns to one slot per whole degree, so a 50mm pillar
subtends about 6 points at 500mm, 3 at 1000mm and ONE at 2000mm. The
winning team's minClusterSize of 10 is unreachable at this resolution and
would reject every pillar we care about. Cluster size therefore does almost
no filtering here; the work is done by two other tests that are stronger
anyway - the point must agree with a camera detection's bearing, and it
must stand clear of every fitted wall.
"""
import math

import numpy as np

from classes.block_map import CAMERA_BEHIND_LIDAR_MM
from classes.wall_sense import scan_points

# ============================================================================
# Association
# ============================================================================
BEARING_WINDOW_DEG = 6.0
# Camera bearing is good to ~2 degrees and the lidar bin is +-0.5, so 6 covers
# both plus mast flex. Widening this past ~8 starts to associate the pillar
# BEHIND the one being looked at, which reads as a sudden 400mm jump outward.

MIN_RANGE_MM = 120.0        # closer than this is the robot's own bodywork
MAX_RANGE_MM = 2200.0       # ObjectSolver already discards past 200cm

# ============================================================================
# Rejecting things that are not pillars
# ============================================================================
WALL_CLEARANCE_MM = 120.0
# Anything nearer a fitted wall line than this IS that wall. 120mm is about
# six times the fit's own residual, so wall returns are rejected decisively.
#
# It is tempting to make this much larger - the winning team's pillar filter
# uses 300mm - but that number does not survive the trip to our geometry.
# The lane is 1000mm wide and the lane table's own setpoints run from 250mm
# to 760mm off the outer wall, so a pillar in an INNER or OUTER slot stands
# roughly 200-300mm from a wall as a matter of course. A 300mm exclusion
# zone deletes exactly the pillars the controller has to see. Keep this
# comfortably below the closest a pillar is ever legally placed.

RANGE_GAP_MM = 120.0
# Points further apart in RANGE than this along the same bearing window are
# different objects. A pillar is 50mm deep, so this is generous on purpose;
# it is separating a pillar from the wall a metre behind it, not resolving
# two pillars that are touching.

PILLAR_HALF_DEPTH_MM = 25.0
# The lidar sees the near FACE of a 50mm box. Slots are defined by the
# pillar's centre, so half a box depth is added back on. Small, but it is a
# consistent bias and the dead-bands are only 150mm wide.

MIN_CLUSTER_POINTS = 1
# Deliberately 1. At 2m a pillar IS one point; requiring more would blind us
# exactly where the corner lookahead needs to see. Trust comes from the
# bearing match and the wall-clearance test, not from the count.


class PillarFix:
    """A camera detection with its range re-measured, or passed through."""

    __slots__ = ("color", "bearing_deg", "range_mm", "source", "points")

    def __init__(self, color, bearing_deg, range_mm, source, points=0):
        self.color = color
        self.bearing_deg = float(bearing_deg)
        self.range_mm = float(range_mm)
        self.source = source            # "lidar" or "camera"
        self.points = int(points)       # lidar returns backing the fix

    @property
    def trusted(self):
        """True when the range came from the lidar rather than box height."""
        return self.source == "lidar"

    def position(self):
        """(x, y) in the lidar frame: x to the right, y straight ahead."""
        a = math.radians(self.bearing_deg)
        return (self.range_mm * math.sin(a), self.range_mm * math.cos(a))

    def __repr__(self):
        return (f"PillarFix({self.color} {self.bearing_deg:+.0f}deg "
                f"{self.range_mm:.0f}mm {self.source}/{self.points})")


# ============================================================================
# GEOMETRY
# ============================================================================
def camera_bearing(x, y):
    """
    Bearing of a lidar-frame point as the CAMERA would see it, in degrees.

    The camera stands CAMERA_BEHIND_LIDAR_MM behind the lidar, so in the
    lidar frame it sits at (0, -170) and the vector to the point is
    (x, y + 170). Getting this backwards makes near pillars associate to
    the wrong detection and far ones to none at all.
    """
    return math.degrees(math.atan2(x, y + CAMERA_BEHIND_LIDAR_MM))


def clear_of_walls(x, y, walls, clearance_mm=WALL_CLEARANCE_MM):
    """
    Is this point far enough from every fitted wall to be a pillar?

    Measured to the INFINITE line of each wall, which is what we want: a
    point beyond the end of the fitted segment but still in line with it is
    almost always more of that same wall that the fit did not reach.
    """
    if walls is None:
        return True
    for name in ("front", "back", "left", "right", "far_left", "far_right"):
        wall = getattr(walls, name, None)
        if wall is None:
            continue
        if wall.perpendicular_distance(x, y) < clearance_mm:
            return False
    return True


# ============================================================================
# THE FIX
# ============================================================================
def _candidates(points, walls):
    """Lidar points that could be a pillar, as (bearing_cam, range, x, y)."""
    out = []
    for x, y in points:
        r = math.hypot(x, y)
        if not (MIN_RANGE_MM <= r <= MAX_RANGE_MM):
            continue
        if not clear_of_walls(x, y, walls):
            continue
        out.append((camera_bearing(x, y), r, x, y))
    return out


def _nearest_cluster(matches):
    """
    Group range-sorted matches and return the nearest group.

    Nearest rather than largest: a pillar occludes whatever is behind it, so
    the first thing along the bearing IS the pillar. Taking the biggest
    cluster would prefer the wall behind it, which is exactly the mistake
    that makes a controller ignore an obstacle it can plainly see.
    """
    if not matches:
        return None
    matches = sorted(matches, key=lambda m: m[1])
    group = [matches[0]]
    for m in matches[1:]:
        if m[1] - group[-1][1] > RANGE_GAP_MM:
            break
        group.append(m)
    if len(group) < MIN_CLUSTER_POINTS:
        return None
    return group


def refine(scan, detections, walls=None, window_deg=BEARING_WINDOW_DEG):
    """
    Re-range each camera detection against the lidar.

    I/O:
        scan: 360-element lidar scan in mm, indexed by whole degrees
        detections: ObjectSolver DetectedObjects (colour, bearing_deg,
            distance_cm); only those three fields are read
        walls: a wall_sense.ResolvedWalls, used to reject wall returns.
            None skips that test, which is much weaker - pass them.
        window_deg: bearing half-window for association
        return: one PillarFix per detection, in the same order. A fix whose
            .trusted is False fell back to the camera's own range.

    A lidar point claimed by two detections of DIFFERENT colours is dropped
    from both: the association is ambiguous, and a pillar's colour decides
    which side we pass it on, so guessing costs a wall strike.
    """
    detections = list(detections)
    if not detections:
        return []

    points = scan_points(scan)
    cands = _candidates(points, walls) if len(points) else []

    # Which detection each candidate matches - and whether more than one
    # colour lays claim to it.
    claims = [[] for _ in cands]
    for di, det in enumerate(detections):
        for ci, (bearing, _, _, _) in enumerate(cands):
            if abs(bearing - float(det.bearing_deg)) <= window_deg:
                claims[ci].append(di)

    contested = set()
    for ci, owners in enumerate(claims):
        colors = {detections[d].color for d in owners}
        if len(colors) > 1:
            contested.add(ci)

    fixes = []
    for di, det in enumerate(detections):
        matches = [cands[ci] for ci, owners in enumerate(claims)
                   if di in owners and ci not in contested]
        group = _nearest_cluster(matches)
        if group is None:
            fixes.append(PillarFix(det.color, det.bearing_deg,
                                   float(det.distance_cm) * 10.0, "camera"))
            continue
        # Mean of the near face, plus half a box to reach the centre.
        face = float(np.mean([m[1] for m in group]))
        fixes.append(PillarFix(det.color, det.bearing_deg,
                               face + PILLAR_HALF_DEPTH_MM, "lidar", len(group)))
    return fixes

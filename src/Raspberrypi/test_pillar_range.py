"""
Does the lidar actually improve on the camera's range guess?

The claim in the plan is that monocular range is too coarse for slot
classification (+-150mm) and that the lidar fixes it without touching the
vision pipeline. That is a measurable claim, so it is measured here.

Scans are synthesised from FieldMap plus discs standing in for pillars, so
the true range is known exactly. Camera detections are faked with a
DELIBERATELY WRONG range - the error monocular ranging really makes - and
the test checks the refiner throws that number away.

    uv run python test_pillar_range.py
"""
import math
import sys

import numpy as np

from classes.field_map import FieldMap
from classes.pillar_range import PILLAR_HALF_DEPTH_MM, camera_bearing, refine
from classes.wall_sense import resolve_walls
from utils.enums import Color
from test_wall_sense import synth_scan

BOX_MM = 50.0


class FakeDetection:
    """Duck-types ObjectSolver.DetectedObject: only three fields are read."""

    def __init__(self, color, bearing_deg, distance_cm):
        self.color = color
        self.bearing_deg = bearing_deg
        self.distance_cm = distance_cm


def to_robot(rx, ry, heading_deg, px, py):
    """A field point in the robot frame: x right, y ahead."""
    a = math.radians(heading_deg)
    dx, dy = px - rx, py - ry
    return (dx * math.cos(a) - dy * math.sin(a),
            dx * math.sin(a) + dy * math.cos(a))


def check(name, ok, detail=""):
    print(f"  {name:<50} {'ok ' if ok else 'BAD'} {detail}")
    return bool(ok)


def main():
    field = FieldMap()
    passed = total = 0
    rx, ry, heading = 0.0, -1000.0, 90.0        # south straight, driving east

    # ------------------------------------------------------------------
    # A pillar at a known place, with the camera reporting a range that is
    # 25% too far - about what a half-clipped box produces.
    # ------------------------------------------------------------------
    print("\nRange recovered from a camera guess that is 25% too far")
    print(f"  {'true':>7}  {'camera said':>12}  {'lidar fix':>10}  {'error':>7}")
    for pillar in ((300.0, -1100.0), (600.0, -900.0), (900.0, -1250.0),
                   (1100.0, -700.0)):
        x, y = to_robot(rx, ry, heading, *pillar)
        truth = math.hypot(x, y)
        scan = synth_scan(field, rx, ry, heading, pillars=[pillar])
        walls = resolve_walls(scan, heading, heading)
        det = FakeDetection(Color.RED, camera_bearing(x, y), truth * 1.25 / 10.0)
        fix = refine(scan, [det], walls)[0]
        err = fix.range_mm - truth
        total += 1
        ok = fix.trusted and abs(err) <= 60.0
        passed += ok
        print(f"  {truth:7.0f}  {truth * 1.25:12.0f}  {fix.range_mm:10.0f}  "
              f"{err:+7.0f}   {'ok' if ok else 'BAD'} ({fix.points}pts)")

    # ------------------------------------------------------------------
    # The wall-clearance test is what stops a wall return being read as a
    # pillar. With no pillar present at all, there must be no lidar fix.
    # ------------------------------------------------------------------
    print("\nNo pillar there: must fall back, not invent one")
    scan = synth_scan(field, rx, ry, heading)
    walls = resolve_walls(scan, heading, heading)
    for bearing in (-20.0, 0.0, 20.0, 45.0):
        det = FakeDetection(Color.GREEN, bearing, 80.0)
        fix = refine(scan, [det], walls)[0]
        total += 1
        passed += check(f"bearing {bearing:+.0f} with an empty field",
                        not fix.trusted, f"source={fix.source}")

    # ------------------------------------------------------------------
    # Two pillars, one behind the other on nearly the same bearing. The
    # NEAR one is the one being looked at.
    # ------------------------------------------------------------------
    print("\nTwo pillars in line: the near one wins")
    near, far = (500.0, -1000.0), (1100.0, -1000.0)
    x, y = to_robot(rx, ry, heading, *near)
    scan = synth_scan(field, rx, ry, heading, pillars=[near, far])
    walls = resolve_walls(scan, heading, heading)
    det = FakeDetection(Color.RED, camera_bearing(x, y), 150.0)
    fix = refine(scan, [det], walls)[0]
    truth = math.hypot(x, y)
    total += 1
    passed += check("ranges the near pillar, not the far one",
                    fix.trusted and abs(fix.range_mm - truth) <= 60.0,
                    f"got {fix.range_mm:.0f}, near={truth:.0f}, far=1100")

    # ------------------------------------------------------------------
    # A point two colours both claim is ambiguous, and a wrong colour means
    # passing on the wrong side. Both detections must decline it.
    # ------------------------------------------------------------------
    print("\nA contested point is dropped by both detections")
    pillar = (700.0, -1000.0)
    x, y = to_robot(rx, ry, heading, *pillar)
    b = camera_bearing(x, y)
    scan = synth_scan(field, rx, ry, heading, pillars=[pillar])
    walls = resolve_walls(scan, heading, heading)
    fixes = refine(scan, [FakeDetection(Color.RED, b, 90.0),
                          FakeDetection(Color.GREEN, b + 1.0, 90.0)], walls)
    total += 1
    passed += check("neither red nor green claims it",
                    not any(f.trusted for f in fixes),
                    f"sources={[f.source for f in fixes]}")

    # Same geometry, same colour twice - that is just a double detection of
    # one pillar and both should range fine.
    fixes = refine(scan, [FakeDetection(Color.RED, b, 90.0),
                          FakeDetection(Color.RED, b + 1.0, 90.0)], walls)
    total += 1
    passed += check("but two detections of ONE colour both range",
                    all(f.trusted for f in fixes),
                    f"sources={[f.source for f in fixes]}")

    # ------------------------------------------------------------------
    # The baseline. The camera is 170mm behind the lidar, so at close range
    # the two sensors genuinely disagree about bearing. Ignoring that is a
    # silent failure, so it is worth showing the size of it.
    # ------------------------------------------------------------------
    print("\nThe 170mm camera/lidar baseline actually matters")
    print(f"  {'range':>7}  {'lidar bearing':>14}  {'camera bearing':>15}  {'diff':>6}")
    for pillar in ((400.0, -1400.0), (400.0, -1000.0), (900.0, -1400.0)):
        x, y = to_robot(rx, ry, heading, *pillar)
        lidar_b = math.degrees(math.atan2(x, y))
        cam_b = camera_bearing(x, y)
        print(f"  {math.hypot(x, y):7.0f}  {lidar_b:14.1f}  {cam_b:15.1f}  "
              f"{cam_b - lidar_b:+6.1f}")
    # And the refiner must still work when the disagreement is largest.
    pillar = (250.0, -1250.0)       # an OUTER slot: 250mm off the outer wall
    x, y = to_robot(rx, ry, heading, *pillar)
    scan = synth_scan(field, rx, ry, heading, pillars=[pillar])
    walls = resolve_walls(scan, heading, heading)
    fix = refine(scan, [FakeDetection(Color.GREEN, camera_bearing(x, y), 99.0)],
                 walls)[0]
    total += 1
    passed += check("close pillar still associates",
                    fix.trusted and abs(fix.range_mm - math.hypot(x, y)) <= 60.0,
                    f"got {fix.range_mm:.0f} want {math.hypot(x, y):.0f}")

    # ------------------------------------------------------------------
    # How many lidar points a pillar is actually worth, at 1 degree bins.
    # This is the constraint that set MIN_CLUSTER_POINTS to 1.
    #
    # The sweep stops at 1300mm because that is where the south straight
    # ends: the robot sits at x=0 and the east wall is at x=1500, so a
    # "pillar" further out than this would be inside the corner, not the
    # lane. Anything beyond belongs to the NEXT segment and is the corner
    # lookahead's problem, not this module's.
    # ------------------------------------------------------------------
    print("\nLidar points per pillar (why the cluster minimum is 1)")
    for dist in (400.0, 700.0, 1000.0, 1200.0, 1300.0):
        pillar = (dist, -1000.0)
        x, y = to_robot(rx, ry, heading, *pillar)
        scan = synth_scan(field, rx, ry, heading, pillars=[pillar])
        walls = resolve_walls(scan, heading, heading)
        fix = refine(scan, [FakeDetection(Color.RED, camera_bearing(x, y), 99.0)],
                     walls)[0]
        subtend = 2.0 * math.degrees(math.atan2(BOX_MM / 2.0, dist))
        total += 1
        ok = fix.trusted and abs(fix.range_mm - dist) <= 80.0
        passed += ok
        print(f"  at {dist:5.0f}mm: subtends {subtend:4.1f}deg, "
              f"{fix.points} pts, fix {fix.range_mm:6.0f}  "
              f"{'ok' if ok else 'BAD'}")

    print(f"\n{passed}/{total} passed")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())

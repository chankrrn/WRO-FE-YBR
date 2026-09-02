"""
Does the wall resolver actually find the walls?

Everything in the new final-round controller is measured against a fitted
wall, so if this is wrong nothing above it can be right. The scans here are
synthesised with FieldMap.raycast from a known pose, which means the true
answer is known exactly and a failure is a real failure rather than a
disagreement between two estimates.

    uv run python test_wall_sense.py

Three properties are worth more than the rest:

    it finds the walls at all, from anywhere on the ring
    it is YAW-INVARIANT - the reason for fitting a line instead of firing a
        ray, so it is tested against the ray it replaces
    a pillar beside the wall does not become the wall - the failure that
        makes a reactive controller swerve at nothing
"""
import math
import sys

import numpy as np

from classes.field_map import FieldMap
from classes.wall_sense import get_lines, resolve_walls

PILLAR_RADIUS_MM = 25.0 * math.sqrt(2.0)    # a 50mm box, worst case across


def synth_scan(field, x, y, heading_deg, pillars=()):
    """
    A 360-entry robot-frame scan from a known pose, indexed by bearing.

    Beam `b` in the robot frame looks along field bearing `heading + b`, which
    is the same convention FieldMap.raycast already takes.
    """
    bearings = np.arange(360, dtype=float)
    angles = np.radians(heading_deg + bearings)
    ranges = field.raycast(x, y, angles).astype(float)
    for px, py in pillars:
        ranges = np.minimum(ranges, _disc_ranges(x, y, angles, px, py))
    return ranges


def _disc_ranges(x, y, angles_rad, px, py):
    """Range to a pillar modelled as a disc, or inf where the ray misses."""
    dx, dy = np.sin(angles_rad), np.cos(angles_rad)
    ox, oy = px - x, py - y
    along = ox * dx + oy * dy                       # projection onto the ray
    perp2 = (ox * ox + oy * oy) - along * along     # squared miss distance
    half = PILLAR_RADIUS_MM ** 2 - perp2
    hit = (half >= 0.0) & (along > 0.0)
    out = np.full(angles_rad.shape, np.inf)
    out[hit] = along[hit] - np.sqrt(half[hit])
    return out


def _keep(scan, bearings):
    """A copy of the scan with every bearing outside `bearings` blanked."""
    out = np.full(360, np.inf)
    idx = list(bearings)
    out[idx] = scan[idx]
    return out


def check(name, got, want, tolerance, notes=""):
    ok = got is not None and not math.isnan(got) and abs(got - want) <= tolerance
    got_text = "none" if got is None or math.isnan(got) else f"{got:7.1f}"
    print(f"  {name:<44} {'ok ' if ok else 'BAD'} "
          f"{got_text} want {want:.0f}+-{tolerance:.0f} {notes}")
    return ok


def main():
    field = FieldMap()
    passed = total = 0

    # ------------------------------------------------------------------
    # The four straights. On each, the robot sits in the middle of the
    # 1000mm lane: 500mm off the outer wall and 500mm off the centre block.
    # ------------------------------------------------------------------
    print("\nThe walls, from the middle of each straight")
    # (name, x, y, heading, which resolved side the OUTER wall lands on)
    straights = [
        ("south, driving east", 0.0, -1000.0, 90.0, "right"),
        ("north, driving west", 0.0, 1000.0, 270.0, "right"),
        ("east, driving north", 1000.0, 0.0, 0.0, "right"),
        ("west, driving south", -1000.0, 0.0, 180.0, "right"),
        ("south, driving west", 0.0, -1000.0, 270.0, "left"),
        ("east, driving south", 1000.0, 0.0, 180.0, "left"),
    ]
    for name, x, y, heading, outer_side in straights:
        scan = synth_scan(field, x, y, heading)
        walls = resolve_walls(scan, heading, heading)
        inner_side = "left" if outer_side == "right" else "right"
        total += 3
        passed += check(f"{name}: outer", walls.distance(outer_side), 500.0, 40.0)
        passed += check(f"{name}: inner", walls.distance(inner_side), 500.0, 40.0)
        passed += check(f"{name}: front", walls.distance("front"), 1500.0, 60.0)

    # ------------------------------------------------------------------
    # Yaw invariance - the whole reason for fitting a line. A single beam
    # fired at 90 degrees reads perpendicular/cos(yaw); the fit should not
    # care. Both are printed so the improvement is visible, not asserted.
    # ------------------------------------------------------------------
    print("\nYaw invariance: fitted wall vs the single ray it replaces")
    print(f"  {'yaw':>5}  {'fitted':>8}  {'err':>7}   {'ray at 90deg':>12}  {'err':>7}")
    for yaw in (-30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0):
        heading = 90.0 + yaw
        scan = synth_scan(field, 0.0, -1000.0, heading)
        walls = resolve_walls(scan, heading, 90.0)
        fitted = walls.distance("right")
        ray = float(scan[90])       # the naive "distance to the wall on my right"
        total += 1
        ok = not math.isnan(fitted) and abs(fitted - 500.0) <= 40.0
        passed += ok
        print(f"  {yaw:>5.0f}  {fitted:8.1f}  {fitted - 500.0:+7.1f}   "
              f"{ray:12.1f}  {ray - 500.0:+7.1f}   {'ok' if ok else 'BAD'}")

    # ------------------------------------------------------------------
    # A pillar beside the wall must not BECOME the wall. This is the case
    # that makes get_min_distance() unusable for holding a lane.
    # ------------------------------------------------------------------
    print("\nA pillar standing beside the wall")
    # The robot is on the south straight driving east. Each pillar sits
    # BETWEEN the robot and the outer wall, so it lands inside the arc the
    # side fit reads from - abeam (field x = 0) is bearing 90, dead right.
    for px, offset_mm in ((0.0, 250.0), (0.0, 150.0), (0.0, 100.0),
                          (200.0, 150.0), (400.0, 150.0)):
        pillar = (px, -1500.0 + offset_mm)
        scan = synth_scan(field, 0.0, -1000.0, 90.0, pillars=[pillar])
        walls = resolve_walls(scan, 90.0, 90.0)
        # What a sector query - lidar_manager.get_min_distance() - would say.
        naive = float(np.min(scan[45:136]))
        bearing = math.degrees(math.atan2(500.0 - offset_mm, px))
        total += 1
        passed += check(f"pillar {offset_mm:.0f}mm off the wall, "
                        f"bearing {bearing:.0f}deg",
                        walls.distance("right"), 500.0, 60.0,
                        f"(a sector query would say: {naive:.0f})")

    # ------------------------------------------------------------------
    # The extremes of the lane. Their controller commands 250mm and 760mm
    # from the outer wall, so the resolver has to stay honest right up
    # against both walls - not just from the comfortable middle.
    # ------------------------------------------------------------------
    print("\nThe extremes of the lane (the lane table's real setpoints)")
    for outer_mm in (230.0, 250.0, 430.0, 620.0, 760.0, 770.0):
        y = -1500.0 + outer_mm
        scan = synth_scan(field, 0.0, y, 90.0)
        walls = resolve_walls(scan, 90.0, 90.0)
        total += 2
        passed += check(f"hugging outer at {outer_mm:.0f}mm: outer",
                        walls.distance("right"), outer_mm, 40.0)
        passed += check(f"hugging outer at {outer_mm:.0f}mm: inner",
                        walls.distance("left"), 1000.0 - outer_mm, 40.0)

    # ------------------------------------------------------------------
    # Approaching a corner: the front wall is the ruler the turn trigger
    # fires on, so it has to stay honest all the way in.
    # ------------------------------------------------------------------
    print("\nThe front wall as a ruler, closing on a corner")
    for x in (0.0, 400.0, 800.0, 1000.0, 1200.0):
        scan = synth_scan(field, x, -1000.0, 90.0)
        walls = resolve_walls(scan, 90.0, 90.0)
        total += 1
        passed += check(f"robot at x={x:+.0f}", walls.distance("front"),
                        1500.0 - x, 60.0)

    # ------------------------------------------------------------------
    # The centre block must never answer "how far to the wall ahead".
    #
    # Its west face is perpendicular to travel, has its normal pointing at
    # the robot and is ~1m long, so it fits as a clean front wall. Taking
    # the FARTHEST front candidate hides that only while the real wall is
    # also visible - and the block occludes all but a ~25deg sliver of it
    # from the south straight, which a single pillar is enough to erase.
    # Read as the front ruler it fires the corner ~2m early, into the block.
    # ------------------------------------------------------------------
    print("\nThe centre block is not the wall ahead")
    # These two together erase the sliver; with either one alone the real
    # wall still fits, which is why the bug only showed on one layout.
    shadow = [(-500.0, -905.0), (500.0, -1085.0)]
    for x, label in ((-1350.0, "deep in the corner square"),
                     (-1200.0, "entering the straight"),
                     (-1000.0, "on the straight")):
        walls = resolve_walls(synth_scan(field, x, -1000.0, 90.0, shadow),
                              90.0, 90.0)
        front = walls.distance("front")
        block = -500.0 - x             # distance to the block's west face
        fooled = front is not None and not math.isnan(front) \
            and abs(front - block) < 60.0
        total += 1
        passed += not fooled
        shown = "none" if front is None or math.isnan(front) else f"{front:.0f}"
        print(f"  {label:<44} {'ok ' if not fooled else 'BAD'} "
              f"front {shown}, block face at {block:.0f}")

    # Rejecting it must not cost us the ruler: the back wall carries the
    # measurement instead, which is the fallback the lap controller uses.
    for x in (-1350.0, -1200.0, -1000.0):
        walls = resolve_walls(synth_scan(field, x, -1000.0, 90.0, shadow),
                              90.0, 90.0)
        total += 1
        passed += check(f"back wall still rules at x={x:+.0f}",
                        walls.distance("back"), x + 1500.0, 60.0,
                        f"-> front {3000.0 - walls.distance('back'):.0f}")

    # And with the sliver intact the true wall still wins outright.
    total += 1
    passed += check("the real wall beats the block when both are visible",
                    resolve_walls(synth_scan(field, -1350.0, -1000.0, 90.0),
                                  90.0, 90.0).distance("front"),
                    2850.0, 60.0)

    # ------------------------------------------------------------------
    # Discard, do not guess. A controller can wait out "I don't know";
    # it cannot survive "confidently wrong". Every degenerate scan below
    # must produce NaN, not a plausible-looking number.
    # ------------------------------------------------------------------
    print("\nDegenerate scans must report nothing, not something")
    good = synth_scan(field, 0.0, -1000.0, 90.0)
    blind = [
        ("empty scan", np.full(360, np.inf)),
        ("all zeros", np.zeros(360)),
        ("all NaN", np.full(360, np.nan)),
        ("only 4 returns", _keep(good, range(88, 92))),
        ("narrow forward cone only",
         _keep(good, list(range(0, 21)) + list(range(340, 360)))),
    ]
    for label, scan in blind:
        walls = resolve_walls(scan, 90.0, 90.0)
        got = walls.distance("right")
        quiet = got is None or math.isnan(got)
        total += 1
        passed += quiet
        print(f"  {label:<44} {'ok ' if quiet else 'BAD'} "
              f"{'none' if quiet else f'{got:.1f}'} want none")

    # The other half of that coin: a wall the robot can only PARTLY see is
    # still a usable wall, because perpendicular_distance() measures to the
    # infinite line. Blanking the arc abeam does not blind the lateral loop.
    occluded = _keep(good, list(range(0, 45)) + list(range(136, 360)))
    total += 1
    passed += check("wall visible only outside the abeam arc",
                    resolve_walls(occluded, 90.0, 90.0).distance("right"),
                    500.0, 40.0, "(extrapolated along the fitted line)")

    # ------------------------------------------------------------------
    # Sanity: the extractor should return a handful of long walls, not
    # dozens of splinters. Splinters mean the merge passes are not working.
    # ------------------------------------------------------------------
    print("\nSegment count (splinters mean the merge passes are broken)")
    scan = synth_scan(field, 0.0, -1000.0, 90.0)
    lines = get_lines(scan)
    total += 1
    ok = 2 <= len(lines) <= 8
    passed += ok
    print(f"  {'segments extracted':<44} {'ok ' if ok else 'BAD'} "
          f"{len(lines)} want 2..8")
    for line in sorted(lines, key=lambda s: -s.length()):
        print(f"      {line}  perp {line.perpendicular_distance():.0f}mm "
              f"bearing {line.perpendicular_bearing():.0f}deg")

    print(f"\n{passed}/{total} passed")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())

"""
Bench test for the parking bay: finding it, and backing into it.

Neither half needs hardware, and neither needs the full round - which is the
point. test_driving.py --parking runs the real task end to end, but it takes a
minute a trial and every failure has to be untangled from the lap driving. This
exercises the two pieces on their own:

    python test_parking.py              both suites
    python test_parking.py --sweep      how much steering lock the park needs

BayFinder is fed real SimLidar scans from known poses, so the clustering is
tested against the same raycast the localizer sees.

ParkingController is driven against a plain bicycle model with a PERFECT pose.
That is deliberate: it isolates the manoeuvre's geometry from the localizer's
error, so a failure here is a fault in the trajectory rather than in the
filter. The full-sim run is what adds pose noise back.
"""
import argparse
import math
import sys

import numpy as np

from classes.field_map import FieldMap
from classes.navigation_manager import Pose
from classes.parking import (BODY_FRONT_MM, BODY_HALF_WIDTH_MM, BODY_REAR_MM,
                             BayFinder, BayFrame, ParkingController,
                             bay_interior, wall_heading_of, wall_rects)

WHEELBASE_MM = 165.0
MAX_COMMAND = 40.0
STEP_S = 0.02
MM_PER_S_AT_FULL = 390.0     # measured with a tape, 16:28 gearing
LINE_DEPTH_MM = 600.0


# ============================================================================
# Finding the bay
# ============================================================================
def check_detection(section, truth_mm, drive, label, tolerance_mm=40.0):
    field = FieldMap()
    field.set_obstacles(wall_rects(section, truth_mm, field))
    from test_navigation import SimLidar

    lidar = SimLidar(field, seed=7)
    finder = BayFinder(field, section)
    found_after = None
    for index, (x, y, heading) in enumerate(drive):
        lidar.set_pose(x, y, heading)
        if finder.observe(Pose(x, y, heading, confidence=0.9), lidar.get_scan()):
            found_after = found_after or index + 1

    if finder.bay is None:
        print(f"  {label:32} NOT FOUND   {finder.status_line()}")
        return False
    error = finder.bay[1] - truth_mm
    ok = abs(error) <= tolerance_mm
    print(f"  {label:32} {'ok ' if ok else 'BAD'} centre {finder.bay[1]:+7.1f} "
          f"(err {error:+5.1f}mm) width {finder.bay_mm:3.0f}mm after {found_after} scans")
    return ok


def straight_along(section, field, offset_mm, forward=True):
    """Poses driving the length of a section, on the racing line."""
    axis = 0 if section in ("south", "north") else 1
    sign = -1.0 if section in ("south", "west") else 1.0
    heading = wall_heading_of(section) if forward else (wall_heading_of(section) + 180.0) % 360.0
    span = np.arange(-1000.0, 1001.0, 50.0)
    if not forward:
        span = span[::-1]
    poses = []
    for value in span:
        point = [0.0, 0.0]
        point[1 - axis if axis == 0 else 0] = 0.0
        point[axis] = value
        point[1 - axis] = sign * (field.outer - offset_mm)
        poses.append((point[0], point[1], heading))
    return poses


def detection_suite():
    print("Finding the bay (SimLidar scans, real clustering)")
    field = FieldMap()
    results = []
    for section in ("south", "east", "north", "west"):
        for truth in (-250.0, 200.0):
            results.append(check_detection(
                section, truth, straight_along(section, field, LINE_DEPTH_MM),
                f"{section} bay at {truth:+.0f}"))
    results.append(check_detection(
        "south", 200.0, straight_along("south", field, LINE_DEPTH_MM, forward=False),
        "south, driven the other way"))

    # An empty field must not produce a bay.
    from test_navigation import SimLidar

    finder = BayFinder(field, "south")
    lidar = SimLidar(FieldMap(), seed=7)
    for x, y, heading in straight_along("south", field, LINE_DEPTH_MM):
        lidar.set_pose(x, y, heading)
        finder.observe(Pose(x, y, heading, confidence=0.9), lidar.get_scan())
    ok = finder.bay is None
    print(f"  {'empty field invents nothing':32} {'ok ' if ok else 'BAD'} "
          f"{finder.status_line()}")
    results.append(ok)
    return results


# ============================================================================
# Backing in
# ============================================================================
def body_corners(x, y, heading_deg):
    radians = math.radians(heading_deg)
    forward = (math.sin(radians), math.cos(radians))
    right = (math.cos(radians), -math.sin(radians))
    return [(x + along * forward[0] + across * right[0],
             y + along * forward[1] + across * right[1])
            for along, across in ((BODY_FRONT_MM, -BODY_HALF_WIDTH_MM),
                                  (BODY_FRONT_MM, BODY_HALF_WIDTH_MM),
                                  (-BODY_REAR_MM, BODY_HALF_WIDTH_MM),
                                  (-BODY_REAR_MM, -BODY_HALF_WIDTH_MM))]


def drive_into_bay(section, centre_mm, travel_heading, wall_side, lock_deg=40.0,
                   start_before_mm=1600.0, quiet=False, label=None, **tuning):
    field = FieldMap()
    field.set_obstacles(wall_rects(section, centre_mm, field))
    frame = BayFrame(section, centre_mm, field, travel_heading, wall_side)
    radius = WHEELBASE_MM / math.tan(math.radians(lock_deg))
    controller = ParkingController(frame, turn_radius_mm=radius,
                                   line_depth_mm=LINE_DEPTH_MM, **tuning)
    if not quiet:
        print(f"      plan: {controller.summary()}")

    axis = frame.wall_axis
    point = [0.0, 0.0]
    point[1 - axis] = centre_mm - start_before_mm * frame.forward
    point[axis] = frame.sign * (field.outer - LINE_DEPTH_MM)
    x, y, heading = point[0], point[1], travel_heading

    for _ in range(30000):
        command = controller.update(Pose(x, y, heading, confidence=0.9), STEP_S,
                                    max_steer=MAX_COMMAND)
        if controller.finished:
            break
        if command is None:
            # The racing line still has the wheel; on this straight that is
            # simply "keep going", but step 0's speed cap still applies.
            cap, _ = controller.path_caps()
            steer, speed = 0.0, int(cap) if cap else 60
        else:
            steer, speed = command
        road = math.radians(max(-lock_deg, min(lock_deg, steer / MAX_COMMAND * lock_deg)))
        velocity = speed / 100.0 * MM_PER_S_AT_FULL
        x += velocity * math.sin(math.radians(heading)) * STEP_S
        y += velocity * math.cos(math.radians(heading)) * STEP_S
        heading = (heading + math.degrees(velocity / WHEELBASE_MM
                                          * math.tan(road) * STEP_S)) % 360.0

    low, high = bay_interior(section, centre_mm, field)
    outside = max(max(low[0] - cx, cx - high[0], low[1] - cy, cy - high[1])
                  for cx, cy in body_corners(x, y, heading))
    error = abs(((heading - wall_heading_of(section) + 180.0) % 360.0) - 180.0)
    error = min(error, 180.0 - error)
    parked = controller.phase == controller.DONE and outside <= 0.0 and error <= 12.0
    if not quiet:
        print(f"  {label or section:32} {'ok ' if parked else 'BAD'} "
              f"{controller.phase:7} margin {-outside:+6.1f}mm  "
              f"off-square {error:4.1f}deg"
              + (f"  [{controller.reason}]" if controller.reason else ""))
    return parked


def manoeuvre_suite():
    print("\nThe four-step park (bicycle model, perfect pose, lock 40deg)")
    cases = [("south", 200.0, 90.0, +1, "south, anticlockwise"),
             ("south", -250.0, 90.0, +1, "south, bay behind the start"),
             ("south", 0.0, 270.0, -1, "south, clockwise (wall on the left)"),
             ("east", -150.0, 0.0, +1, "east"),
             ("north", 300.0, 270.0, +1, "north"),
             ("west", -300.0, 180.0, +1, "west")]
    return [drive_into_bay(s, c, h, w, label=label) for s, c, h, w, label in cases]


def feasibility():
    """
    What the four-step manoeuvre needs in order to close at all.

    Worth printing next to a failing manoeuvre suite, because the failure is
    not a tuning miss: the two arcs move the robot sideways by 2R(1-cos t),
    so a large turning radius forces a long straight in step 3, and the
    closing arc then sweeps the nose forward through the far wall. Neither
    lever below is in this file.
    """
    print("\nWhat the four-step needs (each row: best over turn angle,")
    print("staging depth, park depth and end offset)")
    print(f"\n  {'lock':>5} {'radius':>7}  bay 300mm")
    for lock in (40, 50, 60, 65, 70, 75):
        radius = WHEELBASE_MM / math.tan(math.radians(lock))
        ok = _best_clearance(radius, 300.0)
        print(f"  {lock:5d} {radius:7.1f}  " +
              (f"clears by {ok:.0f}mm" if ok > 0 else "clips the far wall"))
    print(f"\n  {'bay':>5} {'x robot':>8}  at lock 40 (R=197mm)")
    for bay in (300.0, 340.0, 360.0, 400.0):
        ok = _best_clearance(196.6, bay)
        print(f"  {bay:5.0f} {bay / 240.0:8.2f}  " +
              (f"clears by {ok:.0f}mm" if ok > 0 else "clips the far wall"))


def _best_clearance(radius, bay):
    """Best gap to any wall over the manoeuvre's own tunables, in mm."""
    field = FieldMap()
    frame = BayFrame("south", 0.0, field, 90.0, +1)
    best = -1e9
    for stage_d in (280.0, 320.0, 360.0):
        for turn in range(25, 71, 5):
            for park_d in (90.0, 105.0):
                for end in (-40.0, -20.0, 0.0, 20.0):
                    if abs(end) > bay / 2.0 - 120.0:
                        continue
                    c = ParkingController(frame, bay_mm=bay, turn_radius_mm=radius,
                                          turn_deg=float(turn), stage_depth_mm=stage_d,
                                          park_depth_mm=park_d, end_offset_mm=end,
                                          blade_guard_mm=0.0, wall_guard_mm=0.0)
                    if c.straight_mm < 20.0:
                        continue
                    best = max(best, _sweep_clearance(c))
    return best


def _sweep_clearance(c):
    """Smallest wall gap along the manoeuvre this controller would drive."""
    turn = math.radians(c.turn_deg)
    radius, worst = c.turn_radius_mm, 1e9
    s, d, th = c.stage_s_mm, c.stage_depth_mm, 0.0
    legs = [(radius * turn, 1.0 / radius), (c.straight_mm, 0.0),
            (radius * turn, -1.0 / radius)]
    for length, kappa in legs:
        steps = max(1, int(length / 4.0))
        for _ in range(steps):
            step = length / steps
            th_r = math.radians(th)
            s -= math.cos(th_r) * step
            d -= math.sin(th_r) * step
            th += math.degrees(kappa * step)
            worst = min(worst, _gap(c, s, d, th))
    return worst


def _gap(c, s, d, theta):
    """Signed clearance: negative once the chassis is into a wall."""
    body = list(c.frame.corners_local(s, d, theta))
    gap = min(p[1] for p in body)
    half = c.bay_mm / 2.0
    for sign in (-1.0, 1.0):
        box = (sign * half, sign * (half + 10.0), 0.0, 200.0)
        box = (min(box[0], box[1]), max(box[0], box[1]), box[2], box[3])
        if c._overlaps(body, box):
            return -1.0
        gap = min(gap, _box_gap(body, box))
    return gap


def _box_gap(body, box):
    corners = [(box[0], box[2]), (box[1], box[2]), (box[1], box[3]), (box[0], box[3])]
    best = 1e9
    for i in range(4):
        for j in range(4):
            best = min(best, _seg_gap(body[i], body[(i + 1) % 4],
                                      corners[j], corners[(j + 1) % 4]))
    return best


def _seg_gap(p, q, a, b):
    def pt_seg(pt, u, v):
        ux, uy = v[0] - u[0], v[1] - u[1]
        span = ux * ux + uy * uy
        h = 0.0 if span == 0 else max(0.0, min(1.0, ((pt[0] - u[0]) * ux
                                                     + (pt[1] - u[1]) * uy) / span))
        return math.hypot(pt[0] - (u[0] + h * ux), pt[1] - (u[1] + h * uy))
    return min(pt_seg(p, a, b), pt_seg(q, a, b), pt_seg(a, p, q), pt_seg(b, p, q))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sweep", action="store_true",
                        help="also report what the manoeuvre needs in order to close")
    args = parser.parse_args()

    results = detection_suite() + manoeuvre_suite()
    if args.sweep:
        feasibility()
    print(f"\n{sum(results)}/{len(results)} passed")
    raise SystemExit(0 if all(results) else 1)

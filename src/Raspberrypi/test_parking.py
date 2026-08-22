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
MM_PER_S_AT_FULL = 700.0
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
                   start_before_mm=1200.0, quiet=False, label=None, **tuning):
    field = FieldMap()
    field.set_obstacles(wall_rects(section, centre_mm, field))
    frame = BayFrame(section, centre_mm, field, travel_heading, wall_side)
    radius = WHEELBASE_MM / math.tan(math.radians(lock_deg))
    controller = ParkingController(frame, turn_radius_mm=radius,
                                   line_depth_mm=LINE_DEPTH_MM, **tuning)

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
        # None means the racing line still has the wheel; on this straight
        # that is simply "keep going".
        steer, speed = (0.0, 60) if command is None else command
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
    print("\nBacking in (bicycle model, perfect pose, lock 40deg)")
    cases = [("south", 200.0, 90.0, +1, "south, anticlockwise"),
             ("south", -250.0, 90.0, +1, "south, bay behind the start"),
             ("south", 0.0, 270.0, -1, "south, clockwise (wall on the left)"),
             ("east", -150.0, 0.0, +1, "east"),
             ("north", 300.0, 270.0, +1, "north"),
             ("west", -300.0, 180.0, +1, "west")]
    return [drive_into_bay(s, c, h, w, label=label) for s, c, h, w, label in cases]


def lock_sweep():
    print("\nHow much steering lock the manoeuvre needs (south bay)")
    print("  the bay is 1.25 robot lengths, so the turning circle is not a "
          "preference - below\n  about 70deg of road-wheel angle the entry "
          "cannot be driven at all.\n")
    for lock in (30, 35, 40, 45, 50, 55, 60, 70, 80):
        radius = WHEELBASE_MM / math.tan(math.radians(lock))
        parked = drive_into_bay("south", 200.0, 90.0, +1, lock_deg=lock, quiet=True)
        print(f"  lock {lock:2}deg  turning radius {radius:5.0f}mm   "
              f"{'PARKS' if parked else 'fails'}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sweep", action="store_true",
                        help="also report how much steering lock parking needs")
    args = parser.parse_args()

    results = detection_suite() + manoeuvre_suite()
    if args.sweep:
        lock_sweep()
    print(f"\n{sum(results)}/{len(results)} passed")
    raise SystemExit(0 if all(results) else 1)

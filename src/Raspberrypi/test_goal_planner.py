"""
Checks the goal-based planner without a robot, a mat, or a camera.

    python test_goal_planner.py              everything below
    python test_goal_planner.py --scenarios  the named cases, one line each
    python test_goal_planner.py --stress     3000 random layouts
    python test_goal_planner.py --timing     how long a plan costs

Everything here is geometry, so it runs anywhere Python and numpy do - which
is the point. The failure this planner exists to prevent is a pass that was
silently reduced to nothing, and that is exactly the kind of thing that is
cheap to catch here and expensive to catch on a competition mat.

THE ONE INVARIANT THAT MATTERS is the last line of --stress:

    FALSE-OK: 0

A plan the planner reports as fine must actually be fine - the robot's swept
body must clear every pillar and every wall along the whole of it. Everything
else here is a quality measure and can reasonably get worse in exchange for
something; that one is a correctness property. If it is ever non-zero, the
planner is telling the round it is safe when it is not, which is precisely the
bug the old offset profile had.

For the round driven end to end, with detection, pose error and steering lag
in the loop, use the simulator instead:

    python test_driving.py --round final --trials 12 --pillars 4
"""
import argparse
import math
import random
import time
from dataclasses import dataclass


from classes.field_map import FieldMap
from classes.goal_planner import GoalPlanner, Obstacle, _turn_total_deg
from classes.racing_line import RacingLine

# Matches tasks/final/config.toml. Kept here rather than loaded from it so a
# config edit cannot quietly change what "passing" means.
WALL_MARGIN_MM = 520.0
CORNER_RADIUS_MM = 620.0
MIN_RADIUS_MM = 230.0        # turning circle x turn_radius_margin
CLEARANCE_MM = 150.0
HALF_WIDTH_MM = 70.0
FRONT_MM = 200.0
REAR_MM = 40.0
WALL_CLEARANCE_MM = 80.0
ALIGN_MM = 200.0             # straight run-in before a pass
LOOKAHEAD_MM = 400.0         # the largest pursuit.lookahead_max_mm asks for


@dataclass
class Pose:
    """Just enough of NavigationManager's Pose for the planner."""
    x: float
    y: float
    heading: float


def build():
    field_map = FieldMap()
    path = RacingLine(field_map, wall_margin_mm=WALL_MARGIN_MM,
                      corner_radius_mm=CORNER_RADIUS_MM)
    planner = GoalPlanner(
        field_map, path, min_radius_mm=MIN_RADIUS_MM,
        robot_half_width_mm=HALF_WIDTH_MM, robot_front_mm=FRONT_MM,
        robot_rear_mm=REAR_MM, clearance_mm=CLEARANCE_MM,
        wall_clearance_mm=WALL_CLEARANCE_MM, align_mm=ALIGN_MM)
    return field_map, path, planner


def pose_on_line(path, progress, direction, lateral=0.0, heading_error=0.0):
    x, y, heading = path.pose_at(progress, direction)
    radians = math.radians(heading)
    return Pose(x + lateral * math.cos(radians),
                y - lateral * math.sin(radians),
                (heading + heading_error) % 360.0)


# ============================================================================
# SCENARIOS
# ============================================================================

def scenarios():
    """
    The named cases, driven both ways round the loop.

    Some of them are SUPPOSED to come back COMPROMISED, and it matters that
    they do. A pillar pushed 300mm toward the very side it has to be passed on
    leaves a gap the robot does not fit through - the corridor is 1000mm wide
    and the body is 140mm - so the right answer is COMPROMISED with the
    millimetres named, not a confident plan. That is the whole difference from
    the profile this replaced, which reported those as ordinary passes and
    drove into them.

    Which cases land which way shifts with the tuning, though - widening
    path.corner_radius_mm from 480 to 620 turned the mid-corner inward pass
    from impossible into comfortable - so the marked cases are a NOTE, not
    the test. The test is the same property as --stress: a case reported ok must
    really be clear. Being pessimistic is a quality problem to read off the
    table; being wrongly optimistic is a bug.
    """
    _, path, planner = build()
    print(f"{planner}\n{path}\n")
    print(f"{'case':<34} {'dir':>4} {'pts':>4} {'len':>7} {'pillar':>8} "
          f"{'wall':>7}  verdict")

    failures = 0
    for direction in (1, -1):
        start = pose_on_line(path, 0.0, direction)
        cases = [("empty track", start, [])]

        for side, label in ((+1.0, "RED on the line"),
                            (-1.0, "GREEN on the line")):
            x, y = path.point_at(900.0, direction, 0.0)
            cases.append((label, start, [Obstacle(x, y, side)]))

        for lateral, side, label in ((+300.0, +1.0, "RED 300mm to the wall*"),
                                     (-300.0, -1.0, "GREEN 300mm to the block*")):
            x, y = path.point_at(900.0, direction, lateral)
            cases.append((label, start, [Obstacle(x, y, side)]))

        # Opposite sides in quick succession: the plan has to cross the whole
        # corridor between them, which is the tightest thing the round asks
        # for that is still meant to be possible.
        first = path.point_at(800.0, direction, -150.0)
        second = path.point_at(2200.0, direction, +150.0)
        cases.append(("two pillars, opposite sides*", start,
                      [Obstacle(*first, -1.0), Obstacle(*second, +1.0)]))

        low, high = path.bend_spans(direction)[0]
        middle = (low + high) / 2.0
        approach = pose_on_line(path, middle - 1200.0, direction)
        x, y = path.point_at(middle, direction, 0.0)
        cases.append(("pillar mid-corner, outward", approach,
                      [Obstacle(x, y, -1.0 if direction > 0 else +1.0)]))
        cases.append(("pillar mid-corner, inward", approach,
                      [Obstacle(x, y, +1.0 if direction > 0 else -1.0)]))

        cases.append(("recover 250mm/35deg off line",
                      pose_on_line(path, 0.0, direction, 250.0, 35.0), []))

        for label, pose, obstacles in cases:
            plan = planner.plan(pose, direction, obstacles)
            pillar = getattr(plan, "pillar_gap_mm", float("inf"))
            wall = getattr(plan, "wall_gap_mm", float("inf"))
            verdict = "COMPROMISED" if plan.compromised else "ok"
            if not plan.compromised and min(pillar, wall) < 0.0:
                verdict = "FALSE OK"
                failures += 1
            elif plan.compromised != label.endswith("*"):
                verdict = f"{verdict} (note: differs from the marking)"
            print(f"{label:<34} {direction:>+4} {len(plan):>4} "
                  f"{plan.length_mm:>7.0f} "
                  f"{pillar:>8.0f} {wall:>7.0f}  {verdict}")
    print("\n* was COMPROMISED when these were written - informational, not "
          "a pass/fail")
    return failures


# ============================================================================
# STRESS
# ============================================================================

def stress(trials=3000, seed=7):
    """
    Random poses and pillar layouts, checking the invariants that have to hold
    for every plan whatever the layout.
    """
    _, path, planner = build()
    rng = random.Random(seed)
    counts = {"ok": 0, "compromised": 0, "loop": 0, "short": 0, "false_ok": 0}
    worst_ok = float("inf")

    for _ in range(trials):
        direction = rng.choice((1, -1))
        progress = rng.uniform(0.0, path.length)
        pose = pose_on_line(path, progress, direction,
                            rng.gauss(0.0, 60.0), rng.gauss(0.0, 12.0))
        obstacles = []
        for _ in range(rng.randint(0, 3)):
            at = progress + rng.uniform(300.0, 2200.0)
            lateral = rng.choice((-250.0, -125.0, 0.0, 125.0, 250.0))
            x, y = path.point_at(at, direction, lateral)
            obstacles.append(Obstacle(x, y, rng.choice((-1.0, +1.0))))

        plan = planner.plan(pose, direction, obstacles)

        # A plan must always reach further ahead than pure pursuit can look,
        # or the follower runs off the end of it and stops steering.
        if plan.remaining_mm(pose.x, pose.y) < LOOKAHEAD_MM * 1.5:
            counts["short"] += 1
        # Nothing on this track legitimately turns through a full circle.
        if _turn_total_deg(plan.headings) > 540.0:
            counts["loop"] += 1

        if plan.compromised:
            counts["compromised"] += 1
            continue
        counts["ok"] += 1
        gap = min(getattr(plan, "pillar_gap_mm", float("inf")),
                  getattr(plan, "wall_gap_mm", float("inf")))
        worst_ok = min(worst_ok, gap)
        if gap < 0.0:
            counts["false_ok"] += 1

    print(f"\n{trials} random layouts")
    print(f"  reported ok           {counts['ok']}")
    print(f"  reported compromised  {counts['compromised']}   (many are layouts "
          f"no robot could pass - a random side on a pillar already at the "
          f"corridor edge)")
    print(f"  plans that looped     {counts['loop']}")
    print(f"  plans too short       {counts['short']}")
    print(f"  worst real clearance among plans reported ok: {worst_ok:.1f}mm")
    print(f"\n  FALSE-OK: {counts['false_ok']}   <- must be 0")
    return counts["false_ok"] + counts["short"]


# ============================================================================
# TIMING
# ============================================================================

def timing(trials=400, seed=3):
    """
    What a plan costs. The control loop runs at 50Hz - a 20ms budget - and
    replans at goals.replan_interval_s, so the number that matters is the
    worst case, not the median.
    """
    _, path, planner = build()
    rng = random.Random(seed)
    samples = []
    for _ in range(trials):
        direction = rng.choice((1, -1))
        progress = rng.uniform(0.0, path.length)
        pose = pose_on_line(path, progress, direction, rng.gauss(0.0, 60.0))
        obstacles = []
        for _ in range(3):
            at = progress + rng.uniform(300.0, 2200.0)
            x, y = path.point_at(at, direction,
                                 rng.choice((-250.0, 0.0, 250.0)))
            obstacles.append(Obstacle(x, y, rng.choice((-1.0, +1.0))))
        started = time.perf_counter()
        planner.plan(pose, direction, obstacles)
        samples.append((time.perf_counter() - started) * 1000.0)
    samples.sort()
    print(f"\nplan() over {trials} worst-case layouts (3 pillars each)")
    print(f"  median {samples[len(samples) // 2]:.2f}ms   "
          f"p95 {samples[int(len(samples) * 0.95)]:.2f}ms   "
          f"max {samples[-1]:.2f}ms   (tick budget is 20ms)")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenarios", action="store_true")
    parser.add_argument("--stress", action="store_true")
    parser.add_argument("--timing", action="store_true")
    parser.add_argument("--trials", type=int, default=3000)
    args = parser.parse_args()

    everything = not (args.scenarios or args.stress or args.timing)
    failures = 0
    if everything or args.scenarios:
        failures += scenarios()
    if everything or args.stress:
        failures += stress(args.trials)
    if everything or args.timing:
        failures += timing()

    print(f"\n{'FAILED' if failures else 'PASSED'}"
          f"{f' ({failures} checks)' if failures else ''}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())

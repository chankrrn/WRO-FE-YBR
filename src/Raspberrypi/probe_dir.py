"""
Is the robot driving the way it thinks it is?

The crash probe showed one run deciding "clockwise" from a pose that was a
MIRROR of the truth - estimated at x=+944 when the robot was really at
x=-1046 - and then driving the course backwards with the outer wall on the
side it was not looking at. That is not a controller fault, so it needs
measuring before anything else is tuned.

Truth here is the actual angular sweep about the field centre, integrated
from the sim's own pose: positive is counter-clockwise. Reproduces
run_trials' seeding exactly so the trial indices line up with the summary.
"""
import sys
import math
import random

sys.path.insert(0, ".")
import test_driving as td
from classes.field_map import FieldMap
from utils.task_config import TaskConfig
from tasks.final.task import FinalTask
from classes.racing_line import RacingLine

SWEEP = {}
_advance = td.SimRobot.advance


def advance(self, *a, **kw):
    ang = math.atan2(self.y, self.x)
    prev = SWEEP.get("last")
    if prev is not None:
        d = (ang - prev + math.pi) % (2.0 * math.pi) - math.pi
        SWEEP["total"] = SWEEP.get("total", 0.0) + d
    SWEEP["last"] = ang
    return _advance(self, *a, **kw)


td.SimRobot.advance = advance

count = int(sys.argv[1]) if len(sys.argv) > 1 else 5
seed = 0
config = TaskConfig.load("tasks/final/config.toml")
config.set("laps.goal", 3)
rng = random.Random(seed)
field_map = FieldMap()

print(f"\n{'#':>2} {'start pose':>22} {'declared':>10} {'actually drove':>15}"
      f" {'laps':>5} {'verdict':>8}")
flipped = 0
for index in range(count):
    start = td.random_placement(field_map, rng)
    SWEEP.clear()
    out = td.simulate(FinalTask, config, start, timeout_s=200.0,
                      seed=seed + index, pillars=4)
    swept = math.degrees(SWEEP.get("total", 0.0))
    # RacingLine.direction_name maps the task's sign; +sweep is CCW in
    # standard math convention (x right, y up).
    declared = RacingLine.direction_name(out["direction"])[:4].upper()
    drove = "COUN" if swept > 0 else "CLOC"
    agree = abs(swept) < 45.0 or declared == drove
    flipped += not agree
    verdict = ("CRASH" if out["crashed"] else
               "WRONG" if out["pillars_wrong"] or out["pillars_hit"] else
               "OK" if out["completed"] else "SHORT")
    print(f"{index:>2} ({start[0] / 10:+6.1f},{start[1] / 10:+6.1f})cm"
          f" @{start[2]:5.1f}deg {declared:>10} {drove:>7} {swept:>+7.0f}deg"
          f" {out['laps']:>5.2f} {verdict:>8}"
          f" {'' if agree else '  <-- BACKWARDS'}")

print(f"\n{flipped}/{count} drove the opposite way to the one they declared")

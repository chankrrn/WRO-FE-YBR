"""
With parking out of the way the crashes are gone and the failure is SLOWNESS:
three of five trials time out at 0.50-1.00 laps. Where does the time go?

Tallies, per trial: seconds in each controller state, seconds the lap
controller was not driving at all (the reverse-out safety net and anything
else that takes the wheel), and how far the robot actually got.
"""
import sys
import random
from collections import Counter

sys.path.insert(0, ".")
import test_driving as td
from classes.field_map import FieldMap
from utils.task_config import TaskConfig
from tasks.final.task import FinalTask
from tasks.final import lap_controller as lc

TALLY = Counter()
RUNS = []
_update = lc.LapController.update
STATE = {}


def update(self, dt, walls, heading_deg, now):
    cmd = _update(self, dt, walls, heading_deg, now)
    TALLY[f"state:{self.state}"] += 1
    TALLY["lap ticks"] += 1
    # Longest unbroken stretch in one state, which is what a deadlock looks
    # like however it is caused.
    if STATE.get("name") != self.state:
        if STATE.get("name") is not None:
            RUNS.append((STATE["name"], STATE["ticks"]))
        STATE["name"], STATE["ticks"] = self.state, 0
    STATE["ticks"] += 1
    return cmd


lc.LapController.update = update

config = TaskConfig.load(sys.argv[1] if len(sys.argv) > 1
                         else "tasks/final/config.toml")
config.set("laps.goal", 3)
rng = random.Random(0)
field_map = FieldMap()
print(f"\n{'#':>2} {'laps':>5} {'NORMAL':>8} {'PRE_TURN':>9} {'TURNING':>8}"
      f" {'not driving':>12}   longest single stretch")
for index in range(5):
    start = td.random_placement(field_map, rng)
    TALLY.clear()
    RUNS.clear()
    STATE.clear()
    out = td.simulate(FinalTask, config, start, timeout_s=200.0,
                      seed=index, pillars=4)
    if STATE.get("name") is not None:
        RUNS.append((STATE["name"], STATE["ticks"]))
    hz = 50.0
    total_s = out["elapsed_s"]
    driving_s = TALLY["lap ticks"] / hz
    worst = max(RUNS, key=lambda r: r[1]) if RUNS else ("-", 0)
    print(f"{index:>2} {out['laps']:>5.2f}"
          f" {TALLY['state:NORMAL'] / hz:>7.1f}s"
          f" {TALLY['state:PRE_TURN'] / hz:>8.1f}s"
          f" {TALLY['state:TURNING'] / hz:>7.1f}s"
          f" {total_s - driving_s:>11.1f}s"
          f"   {worst[0]} for {worst[1] / hz:.1f}s")

"""
classify() gets the side right 2571 times to 1 for trial 4's outer pillars,
yet the map committed both of them INNER. So the commit is not a vote of the
evidence - it is a vote of the FIRST three agreeing observations, and the map
is write-once, so everything after is discarded.

If that is right, commits happen at long range, where the estimate is worst,
and the 2500 close-range corrections arrive too late to matter. This logs the
range at every commit and tallies how the evidence went afterwards.
"""
import sys
import random
import math
from collections import Counter, defaultdict

sys.path.insert(0, ".")
import test_driving as td
from classes.field_map import FieldMap
from utils.task_config import TaskConfig
from tasks.final.task import FinalTask
from classes import slot_map as sm

COMMITS = []
AFTER = defaultdict(Counter)
BEFORE = defaultdict(Counter)
LAST = {}
_observe = sm.SlotMap.observe
_classify = sm.classify


def classify(x, y, walls, segment, clockwise):
    out = _classify(x, y, walls, segment, clockwise)
    LAST["range"] = math.hypot(x, y)
    LAST["cell"] = out
    return out


sm.classify = classify


def observe(self, *a, **kw):
    before = set(self._slots)
    out = _observe(self, *a, **kw)
    cell = LAST.get("cell")
    if cell is not None:
        key = (cell[0], cell[1])
        (AFTER if key in before else BEFORE)[key][cell[2].name] += 1
    for key in set(self._slots) - before:
        slot = self._slots[key]
        COMMITS.append((key, slot.side.name, slot.color.name,
                        LAST.get("range", float("nan"))))
    return out


sm.SlotMap.observe = observe

trial = int(sys.argv[1]) if len(sys.argv) > 1 else 4
cfg = sys.argv[2] if len(sys.argv) > 2 else "tasks/final/config.toml"
config = TaskConfig.load(cfg)
config.set("laps.goal", 3)
rng = random.Random(0)
field_map = FieldMap()
for _ in range(trial + 1):
    placement = td.random_placement(field_map, rng)
out = td.simulate(FinalTask, config, placement, timeout_s=200.0,
                  seed=trial, pillars=4)

print(f"\ntrial {trial}  laps {out['laps']:.2f}")
print(f"\n{'cell':>10} {'committed':>10} {'at range':>9}   "
      f"votes BEFORE the commit -> votes AFTER it (all discarded)")
for (seg, loc), side, color, rng_mm in COMMITS:
    key = (seg, loc)
    b = dict(BEFORE[key])
    a = dict(AFTER[key])
    print(f"  seg{seg} {loc.name:>4} {side + '/' + color:>10} "
          f"{rng_mm:>7.0f}mm   {b} -> {a}")

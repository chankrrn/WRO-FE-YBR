"""
Both OUTER pillars in trial 4 committed as INNER, with the along-track bin
exactly right - so it is the `lateral` measurement, not the geometry around
it. distances() has two ways to get it:

    outer.perpendicular_distance(...)        when the outer wall is in view
    LANE_MM - inner.perpendicular_distance() when it is not

and the docstring notes the outer wall is occluded exactly when a pillar
stands beside it - which is what an OUTER pillar IS. This logs, per
observation, which branch ran, what it produced, and what the truth was.
"""
import sys
import random
import math
from collections import Counter

sys.path.insert(0, ".")
import test_driving as td
from classes.field_map import FieldMap
from utils.task_config import TaskConfig
from tasks.final.task import FinalTask
from classes import slot_map as sm

TRUTH = {}
SEEN = []
TALLY = Counter()
_advance = td.SimRobot.advance
_classify = sm.classify
_distances = sm.distances
BRANCH = {}


def advance(self, *a, **kw):
    TRUTH.update(x=self.x, y=self.y, h=self.heading)
    return _advance(self, *a, **kw)


td.SimRobot.advance = advance


def distances(x, y, walls, clockwise):
    outer, inner = sm._walls_for(walls, clockwise)
    BRANCH["lateral"] = ("outer wall" if outer is not None
                         else "1000-inner" if inner is not None else "none")
    BRANCH["outer_len"] = None if outer is None else outer.length()
    BRANCH["inner_len"] = None if inner is None else inner.length()
    return _distances(x, y, walls, clockwise)


sm.distances = distances


def classify(x, y, walls, segment, clockwise):
    out = _classify(x, y, walls, segment, clockwise)
    along, lateral = sm.distances(x, y, walls, clockwise)
    # Truth: rotate the robot-frame point back to the field and measure it.
    h = math.radians(TRUTH.get("h", 0.0))
    # Heading is degrees clockwise from +Y, so forward is (sin h, cos h).
    fx, fy = math.sin(h), math.cos(h)
    gx = TRUTH.get("x", 0.0) + y * fx + x * fy
    gy = TRUTH.get("y", 0.0) + y * fy - x * fx
    truth_lateral = 1500.0 - max(abs(gx), abs(gy))
    SEEN.append((BRANCH["lateral"], lateral, truth_lateral,
                 BRANCH["outer_len"], BRANCH["inner_len"],
                 None if out is None else out[2].name))
    TALLY[BRANCH["lateral"]] += 1
    return out


sm.classify = classify

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

print(f"\ntrial {trial}: {len(SEEN)} classify() calls   {dict(TALLY)}")
for branch in ("outer wall", "1000-inner"):
    rows = [r for r in SEEN if r[0] == branch and not math.isnan(r[1])]
    if not rows:
        continue
    errs = sorted(r[1] - r[2] for r in rows)
    n = len(errs)
    print(f"\n  via {branch}: {n} observations")
    print(f"    lateral error  median {errs[n // 2]:+.0f}mm   "
          f"p05 {errs[n // 20]:+.0f}   p95 {errs[-n // 20 - 1]:+.0f}   "
          f"rms {math.sqrt(sum(e * e for e in errs) / n):.0f}mm")
    # Only the OUTER-row pillars matter for the bug being chased.
    outer_rows = [r for r in rows if r[2] < 500.0]
    if outer_rows:
        wrong = sum(1 for r in outer_rows if r[5] == "INNER")
        dead = sum(1 for r in outer_rows if r[5] is None)
        print(f"    of {len(outer_rows)} looks at a REAL OUTER pillar: "
              f"{sum(1 for r in outer_rows if r[5] == 'OUTER')} called OUTER, "
              f"{wrong} called INNER, {dead} discarded")

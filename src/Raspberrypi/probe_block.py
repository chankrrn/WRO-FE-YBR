"""
Three trials spend half their run in PRE_TURN, shuttling forward and back
against something. The wall resolver rightly refuses to call it a wall
(MIN_WALL_MM), and the raw front-clearance sector stops the robot at 130mm,
so it stalls. What is it?

Prints, over the stalled stretch, the robot's truth pose and the truth
position of every pillar with its side and colour - so the thing in the way
can be named rather than guessed at.
"""
import sys
import random
import math

sys.path.insert(0, ".")
import test_driving as td
from classes.field_map import FieldMap
from utils.task_config import TaskConfig
from tasks.final.task import FinalTask
from tasks.final import lap_controller as lc

ROWS = []
TRUTH = {}
PLACED = []
_update = lc.LapController.update
_advance = td.SimRobot.advance
_sim = td.simulate


def advance(self, *a, **kw):
    TRUTH.update(x=self.x, y=self.y, h=self.heading)
    return _advance(self, *a, **kw)


td.SimRobot.advance = advance


def update(self, dt, walls, heading_deg, now):
    cmd = _update(self, dt, walls, heading_deg, now)
    ROWS.append(dict(t=now, state=self.state, target=self.target_outer_mm,
                     front=cmd.front_mm, outer=cmd.outer_mm,
                     x=TRUTH.get("x", 0.0), y=TRUTH.get("y", 0.0),
                     h=TRUTH.get("h", 0.0), seg=self.segment,
                     targeted=self._targeted))
    return cmd


lc.LapController.update = update

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

pillars = out["pillars"]
print(f"\ntrial {trial}  laps {out['laps']:.2f}")
print("pillars on the field (truth):")
for p in pillars:
    d_wall = 1500.0 - max(abs(p.x), abs(p.y))
    print(f"  ({p.x:+7.0f},{p.y:+7.0f})  {p.color.name:<5} "
          f"{d_wall:4.0f}mm from the outer wall  "
          f"({'OUTER' if d_wall < 500 else 'INNER'} row)  "
          f"closest approach {p.min_clearance_mm:+.0f}mm  "
          f"{'correct side' if p.passed_correctly else 'WRONG SIDE'}")

# The stalled stretch: the longest run of ticks that moves less than 300mm.
best = (0, 0)
i = 0
while i < len(ROWS):
    j = i
    while j < len(ROWS) and math.hypot(ROWS[j]["x"] - ROWS[i]["x"],
                                       ROWS[j]["y"] - ROWS[i]["y"]) < 300.0:
        j += 1
    if j - i > best[1] - best[0]:
        best = (i, j)
    i = max(j, i + 1)

lo, hi = best
print(f"\nlongest stretch inside a 300mm circle: {(hi - lo) / 50.0:.0f}s "
      f"from t={ROWS[lo]['t'] % 1000:.0f}s")
print(f"{'t':>6}{'state':>9}{'seg':>4}{'target':>7}{'outer':>7}{'front':>7}"
      f"{'x':>7}{'y':>7}{'head':>6}   nearest pillar")
for k in range(lo, min(hi, lo + 600), 40):
    r = ROWS[k]
    near = min(pillars, key=lambda p: math.hypot(p.x - r["x"], p.y - r["y"]))
    d = math.hypot(near.x - r["x"], near.y - r["y"])
    bearing = (math.degrees(math.atan2(near.x - r["x"], near.y - r["y"]))
               - r["h"] + 180.0) % 360.0 - 180.0
    f = "none" if r["front"] is None else f"{r['front']:.0f}"
    o = "none" if r["outer"] is None else f"{r['outer']:.0f}"
    print(f"{r['t'] % 1000:>6.0f}{r['state']:>9}{r['seg']:>4}"
          f"{r['target']:>7.0f}{o:>7}{f:>7}"
          f"{r['x']:>7.0f}{r['y']:>7.0f}{r['h']:>6.0f}   "
          f"{near.color.name} {d:.0f}mm at {bearing:+.0f}deg"
          f"{'  <- targeted' if r['targeted'] is not None else ''}")

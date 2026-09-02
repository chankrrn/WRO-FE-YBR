"""
The two remaining crashes are -6.9 and -6.6cm against the OUTER WALL, not a
pillar. The first pass showed two rulers failing together in the run-up:
`outer` climbing 554->788 while the body's real gap fell 385->139, and `front`
dropping 803->212 in a single 20ms tick.

Both are resolver questions, so this dumps every classified wall - bearing,
perpendicular distance, length - alongside ground truth for the ticks before
contact.
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
LAST = {}
_update = lc.LapController.update
_check = td.SimRobot._check_clearance


def check(self):
    _check(self)
    LAST.update(x=self.x, y=self.y, h=self.heading,
                clear=self.min_clearance_mm, crashed=self.crashed)


td.SimRobot._check_clearance = check


def describe(seg):
    if seg is None:
        return None
    return (seg.perpendicular_distance(), seg.length(),
            seg.perpendicular_bearing(0.0, 0.0) % 360.0)


def update(self, dt, walls, heading_deg, now):
    cmd = _update(self, dt, walls, heading_deg, now)
    x, y = LAST.get("x", 0.0), LAST.get("y", 0.0)
    ROWS.append(dict(
        t=now, state=self.state, target=self.target_outer_mm,
        outer=cmd.outer_mm, front=cmd.front_mm, steer=cmd.steer_deg,
        datum=self.heading_direction_deg, h=heading_deg,
        x=x, y=y, clear=LAST.get("clear"), crashed=LAST.get("crashed"),
        truth_outer=1500.0 - max(abs(x), abs(y)),
        walls={k: describe(getattr(walls, k))
               for k in ("front", "back", "left", "right",
                         "far_left", "far_right")}))
    return cmd


lc.LapController.update = update

# Reproduce run_trials' seeding exactly: ONE rng drawn from in sequence, so
# trial N here is trial N in the summary. Seeding a fresh Random(N) instead
# gives a different placement and a different run - which is how the first
# pass ended up diagnosing a direction flip that the real trials do not have.
trial = int(sys.argv[1]) if len(sys.argv) > 1 else 2
config = TaskConfig.load("tasks/final/config.toml")
config.set("laps.goal", 3)
rng = random.Random(0)
field_map = FieldMap()
for _ in range(trial + 1):
    placement = td.random_placement(field_map, rng)
out = td.simulate(FinalTask, config, placement, timeout_s=200.0,
                  seed=trial, pillars=4)

hit = next((i for i, r in enumerate(ROWS) if r["crashed"]), None)
print()
print(f"trial {trial}  laps {out['laps']:.2f}  worst {out['min_clearance_mm']:.0f}mm")
if hit is None:
    print("no crash tick recorded")
    sys.exit()

print(f"\n{'t':>6}{'state':>9}{'outer':>7}{'TRUE':>6}{'err':>7}"
      f"{'front':>7}{'clear':>7}   walls (perp/len/bearing)")
for i in range(max(0, hit - 120), min(hit + 5, len(ROWS)), 8):
    r = ROWS[i]
    f = "none" if r["front"] is None else f"{r['front']:.0f}"
    o = "none" if r["outer"] is None else f"{r['outer']:.0f}"
    err = "" if r["outer"] is None else f"{r['outer'] - r['truth_outer']:+.0f}"
    bits = []
    for k, v in r["walls"].items():
        if v is not None:
            bits.append(f"{k[0] if not k.startswith('far') else k[:5]}"
                        f"={v[0]:.0f}/{v[1]:.0f}/{v[2]:.0f}")
    print(f"{r['t'] % 1000:>6.1f}{r['state']:>9}{o:>7}{r['truth_outer']:>6.0f}"
          f"{err:>7}{f:>7}{r['clear']:>7.0f}   {' '.join(bits)}")

"""
Does a pillar on the field land in the right cell of the lattice?

The geometry cases run END TO END - synthesised lidar scan, real wall
resolver, real camera/lidar fusion, real classifier - because the failure
that matters is a pillar landing one cell over, and only the whole chain
can produce that. The voting cases drive the map directly, since what they
check is bookkeeping rather than geometry.

    uv run python test_slot_map.py

Board layout used throughout: the robot drives EAST along the south wall,
so the outer wall is on its right, which is counter-clockwise. Driving that
way it meets C first and A last, per direction.h.
"""
import math
import sys

from classes.field_map import FieldMap
from classes.pillar_range import camera_bearing, refine
from classes.slot_map import (COMMIT_MARGIN, Location, Side, SlotMap, classify,
                              next_segment, segment_from_heading,
                              VOTES_TO_COMMIT)
from classes.wall_sense import resolve_walls
from utils.enums import Color
from test_pillar_range import FakeDetection, to_robot
from test_wall_sense import synth_scan

CCW = False


def check(name, ok, detail=""):
    print(f"  {name:<50} {'ok ' if ok else 'BAD'} {detail}")
    return bool(ok)


def cell_for(field, robot, heading, pillar, segment, clockwise=CCW):
    """Push one pillar through the whole chain and return its cell."""
    x, y = to_robot(robot[0], robot[1], heading, *pillar)
    scan = synth_scan(field, robot[0], robot[1], heading, pillars=[pillar])
    walls = resolve_walls(scan, heading, heading)
    det = FakeDetection(Color.RED, camera_bearing(x, y), math.hypot(x, y) / 10.0)
    fix = refine(scan, [det], walls)[0]
    fx, fy = fix.position()
    return classify(fx, fy, walls, segment, clockwise), fix


def main():
    field = FieldMap()
    passed = total = 0
    heading = 90.0                       # driving east
    segment = segment_from_heading(heading)

    total += 2
    passed += check("heading 90 is segment 1", segment == 1, f"got {segment}")
    passed += check("counter-clockwise steps segment 1 -> 0",
                    next_segment(segment, CCW) == 0)

    # ------------------------------------------------------------------
    # The current lane. Along-track is read off the wall AHEAD, so a pillar
    # 1000mm from it is the last one reached driving CCW (C), and one
    # 2000mm from it is the first (A).
    # ------------------------------------------------------------------
    print("\nCurrent segment: the six cells, end to end")
    robot = (-1000.0, -1000.0)
    # (field position, expected location, expected side)
    cases = [
        ((500.0, -1250.0), Location.A, Side.OUTER),   # 1000 from front wall
        ((500.0, -750.0), Location.A, Side.INNER),
        ((0.0, -1250.0), Location.B, Side.OUTER),     # 1500 from front wall
        ((0.0, -750.0), Location.B, Side.INNER),
        ((-500.0, -1250.0), Location.C, Side.OUTER),  # 2000 from front wall
        ((-500.0, -750.0), Location.C, Side.INNER),
    ]
    for pillar, want_loc, want_side in cases:
        cell, fix = cell_for(field, robot, heading, pillar, segment)
        ok = cell is not None and cell == (segment, want_loc, want_side)
        total += 1
        passed += ok
        got = "discarded" if cell is None else \
            f"seg{cell[0]} {cell[1].name} {cell[2].name}"
        print(f"  {str(pillar):<20} want {want_loc.name} {want_side.name:<5} "
              f"{'ok ' if ok else 'BAD'} {got}")

    # ------------------------------------------------------------------
    # Around the corner, where the two axes transpose. Distance from OUR
    # outer wall becomes along-track in the NEXT segment, and distance from
    # OUR front wall decides that pillar's side.
    # ------------------------------------------------------------------
    print("\nNext segment: the transposed axes")
    robot = (500.0, -1000.0)
    nxt = next_segment(segment, CCW)
    for pillar, want_loc, want_side in (
            ((1250.0, -500.0), Location.C, Side.OUTER),
            ((800.0, -500.0), Location.C, Side.INNER),
            ((1250.0, 0.0), Location.B, Side.OUTER)):
        cell, fix = cell_for(field, robot, heading, pillar, segment)
        ok = cell is not None and cell == (nxt, want_loc, want_side)
        total += 1
        passed += ok
        got = "discarded" if cell is None else \
            f"seg{cell[0]} {cell[1].name} {cell[2].name}"
        print(f"  {str(pillar):<20} want seg{nxt} {want_loc.name} "
              f"{want_side.name:<5} {'ok ' if ok else 'BAD'} {got}")

    # ------------------------------------------------------------------
    # Dead-bands. A pillar placed in one must be thrown away, not rounded.
    # ------------------------------------------------------------------
    print("\nDead-bands discard rather than round")
    robot = (-1000.0, -1000.0)
    for pillar, why in (((250.0, -1250.0), "along-track gap (1250 from front)"),
                        ((-250.0, -1250.0), "along-track gap (1750 from front)"),
                        ((500.0, -1000.0), "lateral gap (500 from outer)")):
        cell, fix = cell_for(field, robot, heading, pillar, segment)
        total += 1
        passed += check(f"{why}", cell is None,
                        "discarded" if cell is None else str(cell))

    # ------------------------------------------------------------------
    # Voting. Geometry is settled above; this is pure bookkeeping, so the
    # map is driven with a stub instead of a synthesised field.
    # ------------------------------------------------------------------
    print("\nCommitting weighs the evidence, it does not count a streak")

    class StubFix:
        trusted = True

        def __init__(self, pos, color):
            self._pos, self.color = pos, color
            self.bearing_deg, self.range_mm, self.source = 0.0, 0.0, "lidar"

        def position(self):
            return self._pos

    # A stub classifier stands in for the geometry: whatever cell we say.
    class StubWalls:
        front = back = left = right = far_left = far_right = None

    import classes.slot_map as sm
    real_classify = sm.classify
    forced = {"cell": (1, Location.A, Side.OUTER)}
    sm.classify = lambda x, y, w, s, c: forced["cell"]
    try:
        n = VOTES_TO_COMMIT
        m = SlotMap()
        red = StubFix((0.0, 0.0), Color.RED)
        green = StubFix((0.0, 0.0), Color.GREEN)
        for _ in range(n - 1):
            last = m.observe([red], StubWalls(), 1, CCW)
        total += 2
        passed += check(f"{n - 1} observations commit nothing",
                        last == [] and len(m) == 0)
        nth = m.observe([red], StubWalls(), 1, CCW)
        passed += check(f"the {n}th commits",
                        len(nth) == 1 and len(m) == 1)
        total += 1
        passed += check("and it reads back", m.get(1, Location.A).color is Color.RED)

        # A MINORITY IS OUT-VOTED, NOT OBEYED. The old rule restarted the
        # tally on any disagreement, which meant three glitch frames landing
        # together beat twenty-two correct ones that did not - measured, and
        # it cost a whole run.
        m2 = SlotMap()
        for _ in range(n):
            m2.observe([red], StubWalls(), 1, CCW)
        for _ in range(n // 3):
            m2.observe([green], StubWalls(), 1, CCW)
        total += 1
        passed += check("a disagreeing minority does not unseat the leader",
                        len(m2) == 1 and m2.get(1, Location.A).color is Color.RED,
                        f"{n} red then {n // 3} green")

        # ...but an EVEN split is not a leader, and stays unknown.
        m3 = SlotMap()
        for _ in range(n):
            m3.observe([red], StubWalls(), 1, CCW)
            m3.observe([green], StubWalls(), 1, CCW)
        total += 1
        passed += check("a cell split down the middle never commits",
                        len(m3) == 0, f"{n} of each")

        # A gap is not evidence of anything, so it costs nothing.
        m4 = SlotMap()
        for i in range(2 * n):
            m4.observe([red] if i % 3 else [], StubWalls(), 1, CCW)
        total += 1
        passed += check("a missed tick does not throw the tally away",
                        len(m4) == 1)

        # Revision: overwhelming later evidence DOES move a committed cell.
        # Write-once froze a cell the moment it was first believed, which
        # bought stability for pillars already passed and lost the ability to
        # fix a wrong guess about one still ahead - the only kind that steers.
        for _ in range(n * int(COMMIT_MARGIN) + n):
            m.observe([green], StubWalls(), 1, CCW)
        total += 1
        passed += check("a committed cell yields to overwhelming evidence",
                        m.get(1, Location.A).color is Color.GREEN)

        # Heading-rate gate.
        m4 = SlotMap()
        for _ in range(n + 2):
            m4.observe([red], StubWalls(), 1, CCW, heading_rate_deg_s=35.0)
        total += 2
        passed += check("fast yaw suppresses detection entirely", len(m4) == 0)
        passed += check("and the map says so", m4.suppressed)

        # Start-segment OUTER veto: the magenta bay reads as a red pillar.
        m5 = SlotMap(start_segment=1)
        for _ in range(n + 2):
            m5.observe([red], StubWalls(), 1, CCW)
        total += 1
        passed += check("start segment OUTER slots are vetoed", len(m5) == 0)

        # ...but only against RED, which is what magenta can be mistaken for.
        # Vetoing green there threw away the 2477 correct readings of a real
        # green pillar and let 12 misreadings commit the cell the wrong way.
        m5b = SlotMap(start_segment=1)
        for _ in range(n):
            m5b.observe([green], StubWalls(), 1, CCW)
        total += 1
        passed += check("a GREEN pillar in that slot still commits",
                        len(m5b) == 1
                        and m5b.get(1, Location.A).side is Side.OUTER)

        # A vetoed cell must not be decided by whatever survives the veto.
        # Twenty refused OUTER readings and three INNER ones is not evidence
        # of an INNER pillar - it is evidence we may not look here.
        m5c = SlotMap(start_segment=1)
        for i in range(20):
            m5c.observe([red], StubWalls(), 1, CCW)
        forced["cell"] = (1, Location.A, Side.INNER)
        for _ in range(n):
            m5c.observe([red], StubWalls(), 1, CCW)
        total += 1
        passed += check("a vetoed cell ignores the surviving minority",
                        len(m5c) == 0, "20 vetoed OUTER then 3 INNER")

        forced["cell"] = (1, Location.A, Side.INNER)
        m6 = SlotMap(start_segment=1)
        for _ in range(n):
            m6.observe([red], StubWalls(), 1, CCW)
        total += 1
        passed += check("but its INNER slots still commit", len(m6) == 1)

        # A camera-only fix must never reach the map.
        forced["cell"] = (1, Location.B, Side.OUTER)
        m7 = SlotMap()
        loose = StubFix((0.0, 0.0), Color.RED)
        loose.trusted = False
        for _ in range(n + 2):
            m7.observe([loose], StubWalls(), 1, CCW)
        total += 1
        passed += check("an untrusted range never commits", len(m7) == 0)
    finally:
        sm.classify = real_classify

    # ------------------------------------------------------------------
    # The lookahead query the corner logic actually makes.
    # ------------------------------------------------------------------
    print("\nfirst_in() returns the pillar met first, per direction")
    m = SlotMap()
    m._slots[(0, Location.A)] = sm.Slot(0, Location.A, Side.OUTER, Color.RED)
    m._slots[(0, Location.C)] = sm.Slot(0, Location.C, Side.INNER, Color.GREEN)
    total += 2
    passed += check("counter-clockwise meets C first",
                    m.first_in(0, False).location is Location.C)
    passed += check("clockwise meets A first",
                    m.first_in(0, True).location is Location.A)

    print(f"\n{passed}/{total} passed")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())

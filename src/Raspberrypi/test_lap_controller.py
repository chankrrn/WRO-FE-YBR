"""
Can it actually drive a lap?

This closes the loop offline. A bicycle model drives a virtual robot, the
real lidar resolver reads synthesised scans, and the real controller steers
- so a sign error, a bad gain or a corner trigger that fires at the wrong
moment shows up here as the robot leaving the lane, not as a passing unit
test. Nothing is stubbed except the chassis.

    uv run python test_lap_controller.py

What this DOES prove: the cascade is internally consistent, converges, and
the state machine sequences corners correctly.

What it does NOT prove: that a positive steer command turns the real robot
the same way it turns this model. That is `steer_sign`, and it is a bench
measurement - see the plan's verification step 3.
"""
import math
import sys

from classes.field_map import FieldMap
from classes.slot_map import (Location, Side, Slot, SLOT_INNER_MM,
                              SLOT_OUTER_MM, SlotMap, slot_position)
from classes.wall_sense import ResolvedWalls, resolve_walls
from tasks.final.lap_controller import (LANE_BLIND_MM, LANE_CENTRE_MM,
                                        LANE_INNER2_MM, lane_for,
                                        LANE_OUTER2_MM, NORMAL, PRE_TURN,
                                        TURN_FRONT_MM, TURNING, LapController)
from utils.enums import Color
from test_wall_sense import synth_scan

WHEELBASE_MM = 165.0
SPEED_MM_S = 300.0
DT = 0.02                       # 50Hz, the real loop rate
CCW = False
HALF_WIDTH_MM = 80.0


class Sim:
    """A kinematic bicycle on the real field, driven by the real controller."""

    def __init__(self, controller, x, y, heading, pillars=()):
        self.field = FieldMap()
        self.c = controller
        self.x, self.y, self.heading = x, y, heading
        self.pillars = list(pillars)
        self.t = 0.0
        self.track = []

    def step(self):
        scan = synth_scan(self.field, self.x, self.y, self.heading,
                          pillars=self.pillars)
        walls = resolve_walls(scan, self.heading, self.c.heading_direction_deg)
        cmd = self.c.update(DT, walls, self.heading, self.t)

        rate = math.degrees(SPEED_MM_S / WHEELBASE_MM
                            * math.tan(math.radians(cmd.steer_deg)))
        self.heading = (self.heading + rate * DT) % 360.0
        a = math.radians(self.heading)
        self.x += SPEED_MM_S * math.sin(a) * DT
        self.y += SPEED_MM_S * math.cos(a) * DT
        self.t += DT
        self.track.append((self.x, self.y, cmd))
        return cmd

    def run(self, seconds):
        for _ in range(int(seconds / DT)):
            self.step()
        return self.track[-1][2]

    def lane_clearance(self):
        """
        Smallest gap between the chassis and any wall over the whole run.

        Negative means it clipped something. The lane runs from 1500 out to
        500 from centre, so this is the distance to the nearer of the two
        boundaries minus the robot's half width.
        """
        worst = 1e9
        for x, y, _ in self.track:
            outer = 1500.0 - max(abs(x), abs(y))          # to the outer wall
            inner = max(abs(x), abs(y)) - 500.0           # to the centre block
            worst = min(worst, outer - HALF_WIDTH_MM, inner - HALF_WIDTH_MM)
        return worst


def check(name, ok, detail=""):
    print(f"  {name:<50} {'ok ' if ok else 'BAD'} {detail}")
    return bool(ok)


def main():
    passed = total = 0

    # ------------------------------------------------------------------
    # Lateral convergence. Start hard against a wall and see whether the
    # cascade brings the robot to the setpoint - and settles there.
    # ------------------------------------------------------------------
    # Against the controller's OWN setpoint, not a hardcoded 500: this block
    # tests the loop, not the choice, and a fresh segment with nothing
    # committed in it deliberately holds the blind lane rather than centre.
    # The lane-choice tests are further down and assert the numbers.
    print("\nConverging on the lane setpoint from both walls")
    print(f"  {'start':>7}  {'target':>7}  {'after 3s':>9}  {'err':>6}"
          f"  {'overshoot':>9}")
    for start_outer in (250.0, 770.0, 500.0):
        c = LapController(CCW, SlotMap(), start_segment=1)
        c.turn_count = 1                       # past the start-segment override
        sim = Sim(c, -1300.0, -1500.0 + start_outer, 90.0)
        sim.run(3.0)
        aim = c.target_outer_mm
        finals = [1500.0 + y for _, y, _ in sim.track[-30:]]
        settled = sum(finals) / len(finals)
        overshoot = max(abs(1500.0 + y - aim)
                        for _, y, _ in sim.track[len(sim.track) // 3:])
        total += 1
        ok = abs(settled - aim) <= 40.0
        passed += ok
        print(f"  {start_outer:7.0f}  {aim:7.0f}  {settled:9.0f}  "
              f"{settled - aim:+6.0f}  {overshoot:9.0f}   "
              f"{'ok' if ok else 'BAD'}")

    # ------------------------------------------------------------------
    # A pillar in play moves the setpoint, and the robot follows it.
    # ------------------------------------------------------------------
    print("\nA pillar moves the lane setpoint")
    slots = SlotMap()
    # Driving CCW, C is met first. Put a RED pillar on the OUTER side.
    slots._slots[(1, Location.C)] = Slot(1, Location.C, Side.OUTER, Color.RED)
    c = LapController(CCW, slots, start_segment=1)
    c.turn_count = 1
    sim = Sim(c, -1300.0, -1000.0, 90.0)
    seen = set()
    for _ in range(int(2.5 / DT)):
        seen.add(round(sim.step().target_outer_mm))
    total += 1
    passed += check("target moved off lane centre",
                    round(LANE_OUTER2_MM) in seen,
                    f"setpoints seen: {sorted(seen)}")

    slots2 = SlotMap()
    slots2._slots[(1, Location.C)] = Slot(1, Location.C, Side.INNER, Color.GREEN)
    c2 = LapController(CCW, slots2, start_segment=1)
    c2.turn_count = 1
    sim2 = Sim(c2, -1300.0, -1000.0, 90.0)
    seen2 = set()
    for _ in range(int(2.5 / DT)):
        seen2.add(round(sim2.step().target_outer_mm))
    total += 1
    passed += check("and the other way for green/inner",
                    round(LANE_INNER2_MM) in seen2,
                    f"setpoints seen: {sorted(seen2)}")

    # ------------------------------------------------------------------
    # Stickiness. Lane centre is the ONE distance that clears neither an
    # outer pillar nor an inner one, so snapping back to it when a window
    # closes aims the robot at whatever it is still driving past. Driven
    # through _choose_lane directly: what is under test is the bookkeeping,
    # and a sim run would only reach these front distances incidentally.
    # ------------------------------------------------------------------
    print("\nThe lane setpoint holds between pillars")
    sticky = SlotMap()
    # CCW meets C first, so C is the one ~2m from the wall ahead.
    sticky._slots[(1, Location.C)] = Slot(1, Location.C, Side.OUTER, Color.RED)
    c3 = LapController(CCW, sticky, start_segment=1)
    c3.turn_count = 1
    c3._choose_lane(2500.0)
    held = c3.target_outer_mm
    total += 1
    passed += check("a pillar in its window sets the lane",
                    round(held) == round(LANE_OUTER2_MM), f"got {held:.0f}")
    for front, label in ((1900.0, "between windows"),
                         (1200.0, "in an empty slot's window"),
                         (None, "with no front wall at all")):
        c3._choose_lane(front)
        total += 1
        passed += check(f"and it holds {label}",
                        c3.target_outer_mm == held,
                        f"got {c3.target_outer_mm:.0f}, held {held:.0f}")
    # 180 degrees from the datum keeps it in TURNING, which is the state
    # that owns the reset.
    c3._turning(180.0, 100.0)
    total += 1
    passed += check("but the corner puts it back to centre",
                    c3.target_outer_mm == LANE_CENTRE_MM,
                    f"got {c3.target_outer_mm:.0f}")

    # ------------------------------------------------------------------
    # The corner. This is the sequence the centre-wall crashes come from.
    # ------------------------------------------------------------------
    print("\nOne corner, start to finish")
    c = LapController(CCW, SlotMap(), start_segment=1)
    c.turn_count = 1
    sim = Sim(c, -1000.0, -1000.0, 90.0)
    states = []
    # Stop the moment the first corner is complete. Running a fixed number
    # of seconds would roll into the SECOND corner - at 300mm/s, 12s covers
    # more than a 3m segment - and the datum would legitimately have
    # advanced twice by the time the assertion ran.
    for _ in range(int(20.0 / DT)):
        cmd = sim.step()
        if not states or states[-1] != cmd.state:
            states.append(cmd.state)
        if c.turn_count == 2 and cmd.state == NORMAL:
            break
    total += 3
    passed += check("visited NORMAL -> PRE_TURN -> TURNING -> NORMAL",
                    states[:4] == [NORMAL, PRE_TURN, TURNING, NORMAL],
                    f"{states[:4]}")
    passed += check("the heading datum advanced by exactly 90",
                    c.heading_direction_deg == 0.0,
                    f"got {c.heading_direction_deg}")
    passed += check("and the turn was counted", c.turn_count == 2,
                    f"turn_count={c.turn_count}")

    # ------------------------------------------------------------------
    # Four corners without touching anything. The real acceptance test.
    # ------------------------------------------------------------------
    print("\nA full lap on an empty field")
    c = LapController(CCW, SlotMap(), start_segment=1)
    c.turn_count = 1
    sim = Sim(c, -1000.0, -1000.0, 90.0)
    sim.run(45.0)
    clearance = sim.lane_clearance()
    total += 2
    passed += check("completed four corners", c.turn_count >= 5,
                    f"turns={c.turn_count - 1}")
    passed += check("never touched a wall", clearance > 0.0,
                    f"closest approach {clearance:.0f}mm")

    # ------------------------------------------------------------------
    # Driving PAST pillars without hitting them - the reported symptom.
    #
    # The pillars are in the synthesised scan as well as the slot map, so
    # the resolver has to fit walls around them at the same time as the
    # controller dodges them.
    # ------------------------------------------------------------------
    # Where a pillar must physically stand for the ported lane table to
    # clear it. Solved from the table rather than assumed: a pillar on the
    # OUTER side is in play for the 250mm and 620mm lanes, so it has to be
    # at least 105mm (half the chassis plus half the box) away from both.
    print("\nWhat pillar placement the lane table assumes")
    need = HALF_WIDTH_MM + 25.0
    outer_lanes = sorted({lane_for(g, False, CCW) for g in (True, False)})
    inner_lanes = sorted({lane_for(g, True, CCW) for g in (True, False)})
    outer_window = (min(outer_lanes) + need, max(outer_lanes) - need)
    inner_window = (min(inner_lanes) + need, max(inner_lanes) - need)
    total += 2
    passed += check("OUTER pillars must sit 355-480mm off the outer wall",
                    outer_window == (355.0, 515.0),
                    f"table allows {outer_window[0]:.0f}-{outer_window[1]:.0f}, "
                    f"classifier caps it at 480")
    passed += check("INNER pillars must sit 535-655mm off the outer wall",
                    inner_window == (535.0, 655.0),
                    f"table allows {inner_window[0]:.0f}-{inner_window[1]:.0f}")

    print("\nDriving a segment past real pillars")
    # Positions come from slot_position(), the inverse of the classifier the
    # controller reads, rather than from numbers typed in here. If a bin
    # moves, the pillar in this test moves with it - a fixture that carries
    # its own copy of the geometry stops testing the product the first time
    # the geometry changes, and does it silently.
    layouts = [
        ("one outer pillar", [(Location.C, Side.OUTER, Color.RED)]),
        ("one inner pillar", [(Location.A, Side.INNER, Color.GREEN)]),
        ("outer then inner", [(Location.C, Side.OUTER, Color.RED),
                              (Location.A, Side.INNER, Color.GREEN)]),
        ("inner then outer", [(Location.C, Side.INNER, Color.GREEN),
                              (Location.A, Side.OUTER, Color.RED)]),
    ]
    for label, layout in layouts:
        s = SlotMap()
        pillars = []
        for loc, side, color in layout:
            s._slots[(1, loc)] = Slot(1, loc, side, color)
            pillars.append(slot_position(1, loc, side, CCW))
        c = LapController(CCW, s, start_segment=1)
        c.turn_count = 1
        sim = Sim(c, -1350.0, -1000.0, 90.0, pillars=pillars)
        sim.run(9.0)
        gap = min(math.hypot(x - px, y - py) - HALF_WIDTH_MM - 25.0
                  for x, y, _ in sim.track for px, py in pillars)
        walls = sim.lane_clearance()
        total += 1
        ok = gap > 0.0 and walls > 0.0
        passed += ok
        print(f"  {label:<22} {'ok ' if ok else 'BAD'} "
              f"pillar gap {gap:6.0f}mm   wall gap {walls:6.0f}mm")

    # ------------------------------------------------------------------
    # The lookahead: the next segment's first pillar sets the trigger.
    # ------------------------------------------------------------------
    print("\nThe corner trigger comes from the NEXT segment")
    empty = LapController(CCW, SlotMap(), start_segment=1)
    total += 1
    passed += check("no pillar ahead uses the default",
                    empty._trigger_for_next() == TURN_FRONT_MM,
                    f"{empty._trigger_for_next():.0f}mm")

    triggers = {}
    for color in (Color.RED, Color.GREEN):
        for side in (Side.INNER, Side.OUTER):
            s = SlotMap()
            # CCW meets C first in the next segment (segment 0).
            s._slots[(0, Location.C)] = Slot(0, Location.C, side, color)
            lc = LapController(CCW, s, start_segment=1)
            triggers[(color.name, side.name)] = lc._trigger_for_next()
    total += 2
    passed += check("red and green choose different triggers",
                    triggers[("RED", "INNER")] != triggers[("GREEN", "INNER")],
                    str({k: round(v) for k, v in triggers.items()}))
    passed += check("all four combinations are distinct",
                    len(set(triggers.values())) == 4)

    # ------------------------------------------------------------------
    # Guards.
    # ------------------------------------------------------------------
    print("\nGuards")

    class NoWalls:
        front = back = left = right = far_left = far_right = None

    c = LapController(CCW, SlotMap(), start_segment=1)
    c.turn_count = 1
    cmd = c.update(DT, NoWalls(), 90.0, 0.0)
    total += 2
    passed += check("no walls at all: reports the loss", cmd.wall_lost)
    passed += check("and steers on heading alone", abs(cmd.steer_deg) < 1e-9,
                    f"steer={cmd.steer_deg:.2f}")

    # A LARGE heading error plus a wall offset must not sum past 180 and come
    # back with the wrong sign. This is the crash that ended a 4.5-lap run:
    # the sum read -212deg, which is really +148, and the robot held full lock
    # the long way round into the outer wall it was trying to leave.
    #
    # A MODERATE lateral error on purpose. Saturating the wall term would make
    # the offset dominate and the sign would legitimately be the wall's, which
    # tests nothing. 167mm of error is ~30deg of offset: enough to carry a
    # 170deg heading error over the edge, small enough that the short-way
    # answer is unambiguously the heading error's side.
    for error_deg, lateral_mm in ((-170.0, -167.0), (170.0, +167.0)):
        c = LapController(CCW, SlotMap(), start_segment=1)
        c.turn_count = 1                     # empty map -> blind lane
        steer, reported = c._steer(DT, c.target_outer_mm + lateral_mm,
                                   c.heading_direction_deg - error_deg)
        total += 2
        passed += check(f"a {error_deg:+.0f}deg error plus an offset stays "
                        f"inside +-180", abs(reported) <= 180.0 + 1e-9,
                        f"{reported:+.0f}deg")
        # Unwrapped this lands at -+200, so the sign is the tell: the short
        # way round from -170 is further negative, i.e. positive error.
        passed += check("...and steers the short way round, not the long one",
                        math.copysign(1.0, steer) == -math.copysign(1.0, error_deg),
                        f"err {reported:+.0f}deg -> steer {steer:+.1f}deg")

    # The cooldown must debounce a pillar cluster misread as a front wall.
    c = LapController(CCW, SlotMap(), start_segment=1)
    c.turn_count = 1
    c.state = NORMAL
    c._last_pre_turn_at = 100.0
    entered = c._normal(500.0, 101.0)          # 1s later, cooldown is 4s
    total += 2
    passed += check("a second corner inside the cooldown is refused",
                    not entered and c.state == NORMAL)
    passed += check("but allowed once it expires",
                    c._normal(500.0, 105.0) and c.state == PRE_TURN)

    # The start segment's OUTER slots are vetoed, so an empty row there is
    # ignorance, not an empty road: the lane has to clear a pillar we are
    # not allowed to see, which rules out both centre and the outer ease.
    c = LapController(CCW, SlotMap(), start_segment=1)
    c._choose_lane(2500.0)
    total += 3
    passed += check("the start segment keeps clear of unseen outer pillars",
                    c.target_outer_mm == LANE_BLIND_MM,
                    f"{c.target_outer_mm:.0f}mm")
    # The blind lane has to clear BOTH rows, not just the vetoed outer one -
    # an inner pillar that has not committed yet is just as unseen.
    passed += check("...far enough to actually pass either row",
                    min(c.target_outer_mm - SLOT_OUTER_MM,
                        abs(c.target_outer_mm - SLOT_INNER_MM)) >= 125.0,
                    f"{c.target_outer_mm - SLOT_OUTER_MM:.0f}mm outer, "
                    f"{abs(c.target_outer_mm - SLOT_INNER_MM):.0f}mm inner")
    # ...and it does not stiffen the wall term there, which would only hold
    # a guessed lane more firmly.
    passed += check("and does not stiffen the wall term on a guess",
                    not c._stiffened)

    # A committed slot still wins over that default - one marked spot holds
    # one pillar, so an INNER slot proves there is no OUTER pillar with it.
    slots = SlotMap(start_segment=1)
    slots._slots[(1, Location.C)] = Slot(1, Location.C, Side.INNER, Color.RED)
    c = LapController(CCW, slots, start_segment=1)
    c._choose_lane(2500.0)                       # 2500mm: C's window
    total += 1
    passed += check("but a committed slot still overrides it",
                    c.target_outer_mm == 430.0, f"{c.target_outer_mm:.0f}mm")

    # The lateral ruler must not follow a jump. This is the centre-block
    # capture: the resolver hands over the block instead of the outer wall,
    # the reading drops 500mm in one tick, and the robot holds station off
    # the middle of the field.
    class _Wall:
        def __init__(self, mm):
            self.mm = mm

        def perpendicular_distance(self, x=0.0, y=0.0):
            return self.mm

    def _sides(left_mm):
        return ResolvedWalls(left=_Wall(left_mm), right=None)

    c = LapController(True, SlotMap(), start_segment=1)   # clockwise: outer left
    for _ in range(3):
        c._outer_mm(_sides(500.0))
    total += 3
    passed += check("small changes are followed",
                    c._outer_mm(_sides(540.0)) == 540.0)
    jumped = [c._outer_mm(_sides(250.0)) for _ in range(3)]
    passed += check("a 290mm jump is refused at first", all(v == 540.0
                                                            for v in jumped),
                    f"{jumped}")
    settled = [c._outer_mm(_sides(250.0)) for _ in range(4)]
    passed += check("...but accepted once it persists", settled[-1] == 250.0,
                    f"{settled}")

    # The wall term must be off through a turn and flushed on the way back.
    c = LapController(CCW, SlotMap(), start_segment=1)
    c.state = TURNING
    # The datum is 90 here (start_segment=1), so "mid-turn" is a heading far
    # from 90 and "arrived" is one within the 30 degree exit tolerance.
    c._turning(0.0, 200.0)                      # 90 deg off the datum
    total += 3
    passed += check("wall term is disabled mid-turn", not c.wall_pid.active)
    c._turning(90.0, 200.0)                     # arrived
    passed += check("re-enabled with no history on exit",
                    c.wall_pid.active and c.wall_pid.integral == 0.0)
    # Leaving a turn re-arms the debounce. Without this the robot exits
    # still crabbed, reads the wall it just left as the wall ahead, and
    # takes the same corner a second time.
    passed += check("and the cooldown restarts on the way out",
                    not c._normal(300.0, 201.0) and c.state == NORMAL)

    print(f"\n{passed}/{total} passed")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())

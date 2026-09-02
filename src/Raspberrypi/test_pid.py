"""
The three PID behaviours the final-round cascade actually depends on.

Not a test of PID arithmetic - that is textbook. These check the three
places where a naive implementation would quietly ruin a lap:

    it stays clamped, so the heading term cannot ask for more steer than
        the servo has
    the integral does not bank error it can never spend
    re-enabling it after a corner does not fire a stale correction

    uv run python test_pid.py
"""
import sys

from classes.pid import PID


def check(name, ok, detail=""):
    print(f"  {name:<52} {'ok ' if ok else 'BAD'} {detail}")
    return bool(ok)


def main():
    passed = total = 0
    dt = 0.02              # 50Hz, the loop rate the controller runs at

    print("\nInactive by default - a term nobody switched on does nothing")
    pid = PID(1.0)
    total += 2
    passed += check("fresh controller is inactive", not pid.active)
    passed += check("update() while inactive returns 0",
                    pid.update(100.0, dt) == 0.0)

    print("\nOutput clamping")
    pid = PID(2.0, output_min=-46.0, output_max=46.0)   # our max_steer_command
    pid.set_active(True)
    out_hi = pid.update(1000.0, dt)
    out_lo = pid.update(-1000.0, dt)
    total += 2
    passed += check("huge +error clamps to +46", out_hi == 46.0, f"got {out_hi}")
    passed += check("huge -error clamps to -46", out_lo == -46.0, f"got {out_lo}")

    print("\nAnti-windup: the integral cannot bank what it cannot spend")
    pid = PID(0.0, ki=1.0, output_min=-10.0, output_max=10.0)
    pid.set_active(True)
    for _ in range(500):                 # 10s of unreachable setpoint
        pid.update(100.0, dt)
    banked = pid.integral
    # The I term alone may command at most output_max, so the integral is
    # capped at output_max/ki = 10. Unclamped it would have reached 1000.
    total += 1
    passed += check("integral capped at output_max/ki", abs(banked - 10.0) < 1e-9,
                    f"got {banked:.1f}, unclamped would be 1000.0")

    # And it must unwind immediately once the error reverses, rather than
    # spending seconds paying off a debt.
    recovered = None
    for i in range(500):
        pid.update(-100.0, dt)
        if pid.integral <= 0.0:
            recovered = (i + 1) * dt
            break
    total += 1
    passed += check("unwinds on sign reversal", recovered is not None
                    and recovered < 0.25, f"took {recovered}s")

    print("\nReset on re-enable - the corner case, literally")
    pid = PID(1.0, ki=1.0)
    pid.set_active(True)
    for _ in range(100):                 # build up history before the turn
        pid.update(50.0, dt)
    before = pid.integral
    pid.set_active(False)                # wall term off through the turn
    during = pid.update(50.0, dt)
    pid.set_active(True)                 # back on, robot somewhere else now
    total += 3
    passed += check("history existed before the turn", before > 0.0,
                    f"i={before:.2f}")
    passed += check("disabled term contributes nothing", during == 0.0)
    passed += check("re-enable cleared the stale integral", pid.integral == 0.0)

    # Re-enabling must not also fire a derivative spike from the old error.
    pid = PID(0.0, kd=1.0)
    pid.set_active(True)
    pid.update(50.0, dt)
    pid.set_active(False)
    pid.set_active(True)
    first = pid.update(0.0, dt)
    total += 1
    passed += check("no derivative kick on the first tick back", first == 0.0,
                    f"got {first}")

    print("\nDerivative kick when the lane setpoint steps 500 -> 250mm")
    # error jumps by 250mm in one tick; d(error)/dt = 12500 mm/s of nothing.
    on_error = PID(0.0, kd=1.0, output_min=-1e9, output_max=1e9)
    on_meas = PID(0.0, kd=1.0, output_min=-1e9, output_max=1e9,
                  derivative_on_measurement=True)
    on_error.set_active(True)
    on_meas.set_active(True, measurement=500.0)
    on_error.update(0.0, dt)                        # settled at setpoint 500
    on_meas.update(0.0, dt, measurement=500.0)
    kick = on_error.update(250.0, dt)               # setpoint steps to 250
    quiet = on_meas.update(250.0, dt, measurement=500.0)
    total += 2
    passed += check("differentiating error kicks", abs(kick) > 1000.0,
                    f"got {kick:.0f}")
    passed += check("differentiating measurement does not", quiet == 0.0,
                    f"got {quiet:.0f}")

    print("\nGuards")
    pid = PID(1.0)
    pid.set_active(True)
    total += 2
    passed += check("dt <= 0 returns 0 rather than dividing by zero",
                    pid.update(10.0, 0.0) == 0.0)
    pid2 = PID(1.0, ki=1.0)
    pid2.set_active(True)
    pid2.update(50.0, dt)
    pid2.set_gains(2.0, 0.5, 0.0)
    passed += check("set_gains drops history accumulated at the old gains",
                    pid2.integral == 0.0)

    print(f"\n{passed}/{total} passed")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())

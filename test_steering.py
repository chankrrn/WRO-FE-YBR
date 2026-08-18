"""
Measure the robot's steering, instead of guessing at it.

Two numbers in config.toml cannot be derived from first principles and both
cause a weave when they are wrong:

    pursuit.max_road_wheel_deg     what the wheels really do at full lock
    pursuit.lag_compensation_s     how long they take to get there

`max_road_wheel_deg` is the more dangerous of the two, because it SCALES every
steering command. Set it lower than reality and the robot turns more than the
controller asked for, every single tick - which looks exactly like a controller
that over-corrects and hunts, and no amount of lookahead tuning will fix it.

This script drives the robot in the open and works both out from the pose
track, using the localizer that is already running. No protractor needed.

    python test_steering.py --calibrate     road-wheel angle per command
    python test_steering.py --lag           steering response time
    python test_steering.py --all

    python test_steering.py --sweep speed.base 45,55,65,75    tuning trade-offs
                                                              (simulated)

CLEAR SPACE NEEDED. The robot drives arcs of up to ~1m radius for a few
seconds at a time. Put it in the middle of a clear mat, not on a bench.

START BUTTON. The Arduino will not drive the motor until the physical start
button is pressed, so anything here that moves waits for it exactly like a
round does - localize first, show you the pose, then wait for the press. That
ordering is deliberate: you get to see that the robot knows where it is before
you commit to letting it move.
"""
import argparse
import math
import sys
import time

import numpy as np

from classes.task_context import TaskContext
from utils.angle_utils import angle_difference

# Speed the arcs are driven at. This has to clear stiction WITH THE WHEELS
# TURNED, which takes noticeably more torque than rolling straight - a speed
# that drives fine in a straight line can leave the robot sitting still the
# moment any steering goes on. Override with --drive-speed.
CALIBRATION_SPEED = 75

# Short burst above the measurement speed to break stiction, then settle back.
# Without it the robot can refuse to start rolling at all on a low-command arc.
KICK_BONUS = 25
KICK_S = 0.35

SETTLE_S = 0.7          # let the wheels reach the angle before measuring
MEASURE_S = 1.6
SAMPLE_S = 0.05

# These arcs are driven open-loop - nothing is steering around obstacles - so
# the lidar is watched the whole time and the run is cut the moment anything
# gets close. A calibration is not worth a broken robot.
ABORT_CLEARANCE_MM = 280.0
ABORT_SECTOR_DEG = 40.0


def clearance_mm(context):
    """Closest lidar return ahead, or None if the lidar is not answering."""
    if context.lidar is None:
        return None
    distance, _ = context.lidar.get_min_distance(-ABORT_SECTOR_DEG, ABORT_SECTOR_DEG)
    return None if math.isnan(distance) else distance


def kick_off(context, steer_command, speed):
    """
    Breaks stiction with a short burst above the target speed, then settles to
    it. Returns once the wheels should be rolling.
    """
    context.motor.drive(steer_command, min(100, speed + KICK_BONUS))
    time.sleep(KICK_S)
    context.motor.drive(steer_command, speed)


def collect(context, steer_command, speed, seconds):
    """
    Drives at a fixed steering command and logs the pose, stopping early if
    anything comes within ABORT_CLEARANCE_MM of the nose.

    I/O:
        return: (samples, aborted) where samples is
                [(t, x_mm, y_mm, heading_deg), ...]
    """
    context.motor.drive(steer_command, speed)
    samples = []
    started = time.monotonic()
    while time.monotonic() - started < seconds:
        front = clearance_mm(context)
        if front is not None and front < ABORT_CLEARANCE_MM:
            context.motor.drive(0, 0)
            return samples, True
        pose = context.nav.get_pose()
        samples.append((time.monotonic() - started, pose.x, pose.y, pose.heading))
        time.sleep(SAMPLE_S)
    return samples, False


def arc_geometry(samples, wheelbase_mm):
    """
    Road-wheel angle implied by an arc, from the pose track alone.

    A bicycle-model vehicle turns at yaw_rate = v / L * tan(delta), so
    delta = atan(yaw_rate * L / v). Both yaw rate and speed are measured
    here rather than assumed, which is what makes this independent of the
    mm_per_s_at_full calibration - a wrong speed figure cannot bias it.

    I/O:
        return: (road_wheel_deg, speed_mm_s, yaw_rate_deg_s), signed the same
                way as the steering command
    """
    if len(samples) < 4:
        return None, 0.0, 0.0

    times = np.array([s[0] for s in samples])
    points = np.array([(s[1], s[2]) for s in samples])
    headings = np.array([s[3] for s in samples])

    span = times[-1] - times[0]
    if span <= 0.0:
        return None, 0.0, 0.0

    # Unwrap the heading so a lap past 360 does not read as a huge jump back.
    turned = 0.0
    for previous, current in zip(headings, headings[1:]):
        turned += angle_difference(current, previous)
    yaw_rate = turned / span

    travelled = float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
    speed = travelled / span
    if speed < 20.0:
        return None, speed, yaw_rate

    road_wheel = math.degrees(math.atan(math.radians(yaw_rate) * wheelbase_mm / speed))
    return road_wheel, speed, yaw_rate


def probe_link(port=None):
    """
    Is the Arduino listening at all? Opens the port, prints its boot banner and
    the reply to each command. No motion - this only checks the link.
    """
    import serial

    from classes.motor_manager import DEFAULT_BAUD, MotorManager

    port = port or MotorManager.autodetect_port()
    print(f"\nOpening {port} @ {DEFAULT_BAUD}...", flush=True)
    link = serial.Serial(port, DEFAULT_BAUD, timeout=0.5)
    try:
        time.sleep(2.0)          # the port opening resets the board
        print(f"  {link.in_waiting} bytes waiting after reset", flush=True)
        for index in range(3):
            print(f"  banner[{index}] "
                  f"{link.readline().decode(errors='ignore').strip()!r}", flush=True)
        for angle, speed in ((0, 0), (0, 40), (0, 0)):
            link.write(f"{angle},{speed},0\n".encode())
            print(f"  sent {angle},{speed},0 -> "
                  f"{link.readline().decode(errors='ignore').strip()!r}", flush=True)
            time.sleep(0.3)
    finally:
        link.write(b"0,0,0\n")
        link.close()
    print("  link closed", flush=True)


def probe_speed(context, speeds):
    """
    Drives straight at each speed and measures how fast the robot ACTUALLY
    went, from the pose track.

    This is the test to run when a calibration comes back as noise: it
    separates "the motor is not turning" from "the motor turns but my speed
    figure is wrong", and it finds the stiction threshold - the command below
    which the robot does not move at all, which is worth knowing before
    speed.minimum is set under it.
    """
    print(f"\n{'speed':>7} {'measured':>12} {'implied full lock':>19}")
    rows = []
    for speed in speeds:
        context.motor.drive(0, speed)
        time.sleep(SETTLE_S)
        samples, aborted = collect(context, 0, speed, MEASURE_S)
        context.motor.drive(0, 0)
        time.sleep(0.8)

        _, measured, _ = arc_geometry(samples, 165.0)
        note = "  (aborted - wall ahead)" if aborted else ""
        if measured < 20.0:
            print(f"{speed:>7} {measured:>10.0f}mm/s {'-- not moving':>19}{note}")
            continue
        rows.append((speed, measured))
        print(f"{speed:>7} {measured:>10.0f}mm/s {measured / speed * 100:>17.0f}{note}")

    if not rows:
        print("\n  The robot never moved. Either the motor is not being driven at all")
        print("  (does the Arduino gate the motor on the start button?), or every")
        print("  speed tried was below its stiction threshold - retry with --speeds 50,70,90")
        return
    moving = [speed for speed, _ in rows]
    print(f"\n  lowest speed that moved: {min(moving)}  -> keep speed.minimum above it")
    print(f"  startup.mm_per_s_at_full = "
          f"{np.median([m / s * 100 for s, m in rows]):.0f}")


def calibrate(context, wheelbase_mm, max_steer_command, commands,
              speed_command=CALIBRATION_SPEED):
    """
    Drives an arc at each steering command and reports the road-wheel angle
    each one really produced.
    """
    print(f"\nCalibrating steering at speed {speed_command} ({len(commands)} arcs, "
          f"~{len(commands) * (SETTLE_S + MEASURE_S + 1.0):.0f}s)...\n")
    print(f"{'command':>8} {'road wheel':>12} {'implied full lock':>18} "
          f"{'speed':>10} {'yaw rate':>10}")

    rows = []
    for command in commands:
        kick_off(context, command, speed_command)
        time.sleep(SETTLE_S)
        samples, aborted = collect(context, command, speed_command, MEASURE_S)
        context.motor.drive(0, 0)
        time.sleep(0.6)

        road_wheel, speed, yaw_rate = arc_geometry(samples, wheelbase_mm)
        if road_wheel is None:
            reason = "too close to a wall" if aborted else "robot did not move enough"
            print(f"{command:>8} {'--':>12}   ({reason})")
            continue
        full_lock = road_wheel * max_steer_command / command
        rows.append((command, road_wheel, full_lock, speed))
        print(f"{command:>8} {road_wheel:>10.1f}deg {full_lock:>16.1f}deg "
              f"{speed:>8.0f}mm/s {yaw_rate:>8.1f}d/s")

    if not rows:
        print(f"\n  No usable arcs - the robot did not move at speed {speed_command}.")
        print( "  Steering costs more torque than driving straight, so the speed that")
        print( "  rolls fine in a line may not be enough with the wheels turned:")
        print(f"      uv run python test_steering.py --calibrate --drive-speed "
              f"{min(100, speed_command + 20)}")
        print( "  Or skip the arcs entirely - a normal lap measures the same number")
        print( "  from ordinary cornering, at whatever speed the robot really drives:")
        print( "      uv run python main.py qualification --laps 1")
        return

    full_locks = [row[2] for row in rows]
    speeds = [row[3] for row in rows]
    best = float(np.median(full_locks))
    spread = float(np.max(full_locks) - np.min(full_locks))

    print(f"\n  pursuit.max_road_wheel_deg = {best:.1f}")
    print(f"  spread across commands: {spread:.1f}deg", end="")
    if spread > 0.25 * best:
        print("  <- steering is NOT linear; trust the mid-range arcs most")
    else:
        print("  (consistent - the linear model holds)")
    print(f"  startup.mm_per_s_at_full = {np.median(speeds) / speed_command * 100:.0f}"
          f"   (measured at speed {speed_command})")


def measure_lag(context, wheelbase_mm, command=45):
    """
    Steering response time, from a step input.

    Commands a step from centre to `command` and watches how fast the yaw rate
    builds. The time to reach 63% of the final value is the first-order time
    constant - which is what pursuit.lag_compensation_s wants.
    """
    print(f"\nMeasuring steering lag (step to {command})...")
    kick_off(context, 0, CALIBRATION_SPEED)
    time.sleep(1.0)

    samples, _ = collect(context, command, CALIBRATION_SPEED, 2.0)
    context.motor.drive(0, 0)

    times = np.array([s[0] for s in samples])
    headings = np.array([s[3] for s in samples])
    rates = np.array([angle_difference(b, a) / max(dt, 1e-3)
                      for a, b, dt in zip(headings, headings[1:], np.diff(times))])
    if len(rates) < 6:
        print("  not enough samples - is the pose updating?")
        return

    # Smooth, then find when it first crosses 63% of the settled rate.
    smoothed = np.convolve(np.abs(rates), np.ones(3) / 3.0, mode="valid")
    settled = float(np.median(smoothed[-max(3, len(smoothed) // 3):]))
    if settled < 5.0:
        print(f"  yaw rate only reached {settled:.1f}deg/s - too small to time")
        return

    crossing = np.flatnonzero(smoothed >= 0.63 * settled)
    tau = times[crossing[0] + 1] if len(crossing) else float("nan")
    print(f"  settled yaw rate {settled:.1f}deg/s, 63% reached at {tau:.2f}s")
    print(f"\n  pursuit.lag_compensation_s = {min(tau, 0.15):.2f}")
    print("  (capped at 0.15 - over-predicting is worse than not predicting;"
          " if the true lag is larger, cut speed.corner instead)")


def sweep(key, values):
    """
    Simulated trade-off table for one config key. No hardware; this is for
    seeing which direction a knob moves things before trying it for real.
    """
    import argparse as _argparse
    import contextlib
    import io

    from tasks.cli import load_config
    from tasks.qualification.task import QualificationTask
    import test_driving as td

    print(f"\nSimulated sweep of {key} (2 laps, 2 placements each)\n")
    print(f"{key:>22} {'off-line rms':>13} {'worst clearance':>17}  ok")
    for raw in values:
        value = float(raw) if "." in raw or "e" in raw.lower() else int(raw)
        results = []
        for seed in (3, 9):
            config = load_config(QualificationTask, _argparse.Namespace(
                config=None, laps=None, speed=None, corner_speed=None))
            config.set("laps.goal", 2)
            config.set(key, value)
            with contextlib.redirect_stdout(io.StringIO()):
                results.append(td.simulate(QualificationTask, config, (1000.0, 0.0, 0.0),
                                           seed=seed))
        rms = np.mean([r["rms_offset_mm"] for r in results]) / 10
        clear = np.min([r["min_clearance_mm"] for r in results]) / 10
        ok = all(r["completed"] and not r["crashed"] for r in results)
        print(f"{value:>22} {rms:>11.2f}cm {clear:>15.1f}cm  {'yes' if ok else 'NO'}")


def main(args):
    if args.sweep:
        sweep(args.sweep[0], args.sweep[1].split(","))
        return 0

    if args.probe_link:
        probe_link(args.motor_port)
        return 0

    context = TaskContext(debug=False, use_lidar=True, lidar_port=args.lidar_port,
                          motor_port=args.motor_port)
    with context:
        print("Waiting for the localizer...")
        for _ in range(80):
            if context.nav.update().is_reliable:
                break
            time.sleep(0.05)
        pose = context.nav.get_pose()
        print(f"  {pose}")
        if not pose.is_reliable:
            print("  WARNING: pose is not reliable; the numbers below will be noisy")

        front = clearance_mm(context)
        print(f"  {front:.0f}mm clear ahead" if front else "  no lidar reading ahead")
        if front is not None and front < ABORT_CLEARANCE_MM * 2:
            print(f"  Not enough room to drive - need at least "
                  f"{ABORT_CLEARANCE_MM * 2:.0f}mm ahead. Move the robot clear.")
            return 1

        # The Arduino ignores drive commands until the button is pressed, so
        # without this every measurement below reads as "the robot did not
        # move" - which is exactly what it did the first time round.
        if not args.no_wait:
            context.wait_for_start()

        commands = [int(v) for v in args.commands.split(",")]
        try:
            if args.speeds or args.all:
                probe_speed(context, [int(v) for v in (args.speeds or "25,35,50,70").split(",")])
            if args.calibrate or args.all:
                calibrate(context, args.wheelbase, args.max_steer_command, commands,
                          args.drive_speed)
            if args.lag or args.all:
                measure_lag(context, args.wheelbase, max(commands))
        finally:
            context.motor.drive(0, 0)
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Measure the robot's steering geometry")
    parser.add_argument("--calibrate", action="store_true",
                        help="measure the road-wheel angle each command produces")
    parser.add_argument("--lag", action="store_true",
                        help="measure the steering response time")
    parser.add_argument("--all", action="store_true", help="all of the above")
    parser.add_argument("--probe-link", action="store_true",
                        help="check the Arduino answers at all (no motion)")
    parser.add_argument("--speeds", nargs="?", const="25,35,50,70", default=None,
                        metavar="LIST",
                        help="drive straight at each speed and measure the real mm/s")
    parser.add_argument("--sweep", nargs=2, metavar=("KEY", "VALUES"),
                        help="simulated trade-off table, e.g. --sweep speed.base 45,55,65")
    parser.add_argument("--commands", default="20,35,50,65,80",
                        help="steering commands to test (default: 20,35,50,65,80)")
    parser.add_argument("--drive-speed", type=int, default=CALIBRATION_SPEED,
                        help=f"speed to drive the arcs at (default: {CALIBRATION_SPEED}). "
                             f"Raise it if the robot will not move with the wheels turned.")
    parser.add_argument("--wheelbase", type=float, default=165.0)
    parser.add_argument("--max-steer-command", type=float, default=80.0)
    parser.add_argument("--no-wait", action="store_true",
                        help="skip the Arduino start button (only useful if the "
                             "firmware does not gate the motor on it)")
    parser.add_argument("--motor-port", default=None)
    parser.add_argument("--lidar-port", default=None)
    args = parser.parse_args()

    if not (args.calibrate or args.lag or args.all or args.sweep
            or args.probe_link or args.speeds):
        parser.print_help()
        sys.exit(0)
    sys.exit(main(args))

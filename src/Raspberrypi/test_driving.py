"""
Closed-loop test of a whole round, with no hardware.

This runs the REAL task - the real QualificationTask, the real
NavigationManager, the real RacingLine and PurePursuit, the real
tasks/*/config.toml - against a simulated robot and a simulated lidar. The
only fakes are the four things that touch metal: the motor, the lidar, the
compass and the start button.

    python test_driving.py                    one run from a random placement
    python test_driving.py --window           ... with the field view
    python test_driving.py --trials 20        many placements, pass/fail summary
    python test_driving.py --start 900 -1200 45
    python test_driving.py --round final      the obstacle round's config

What it checks, per run:
    * did it complete the laps, and how long did it take
    * how close did it get to a wall or to the centre block (the number that
      matters - anything at or below zero is a crash)
    * how far off the racing line did it drift

Time is virtual. The task's control loop sleeps between ticks, so the sim
hangs the physics off that sleep: every time the round sleeps, the clock jumps
forward and the bicycle model advances by the same amount. A 22m run therefore
takes a second or two of wall clock instead of a minute, and the task under
test cannot tell the difference.
"""
import argparse
import math
import random
import time

import numpy as np

from classes.field_map import FieldMap
from classes.navigation_manager import NavigationManager, Pose
from classes.racing_line import RacingLine
from tasks.cli import load_config
from test_navigation import SimCompass, SimLidar

# ============================================================================
# Simulated robot
# ============================================================================
ROBOT_RADIUS_MM = 100.0      # for the clearance check - half a WRO chassis
PHYSICS_STEP_S = 0.02        # bicycle model integration step

# Steering does not teleport. A hobby servo needs ~0.1-0.2s per 60 degrees and
# the linkage adds more, so the wheels lag the command - which is the classic
# cause of a weave that a lag-free simulator will never show you. Sweep these
# with --servo-lag / --servo-rate to find gains that survive the real thing.
STEER_LAG_S = 0.12           # first-order time constant of the wheels
STEER_RATE_DEG_S = 400.0     # slew limit, in MotorManager command units/s

# Ratio of the robot's TRUE full-lock road-wheel angle to the one in
# config.toml. 1.0 means the config is right. Above 1.0 the robot turns more
# than the controller believes it asked for, which reads as over-compensation.
STEER_GAIN_ERROR = 1.0

# The sim's own idea of how fast the robot goes, kept separate from the
# config's mm_per_s_at_full so a wrong calibration there shows up as the error
# it would really be rather than cancelling itself out.
TRUE_MM_PER_S_AT_FULL = 700.0


class SimMotor:
    """
    MotorManager stand-in. Records the commands; SimRobot integrates them.

    Steering is stored as a command in MotorManager's units and converted to a
    road-wheel angle by SimRobot, so a wrong pursuit.max_road_wheel_deg in the
    config shows up here as the mis-steer it would really cause.
    """

    def __init__(self):
        self.current_speed = 0
        self.current_angle = 0.0
        self.commands = 0

    def forward(self, speed):
        self.current_speed = speed
        self.commands += 1

    def reverse(self, speed):
        self.forward(-abs(speed))

    def stop(self):
        self.current_speed = 0

    def steer(self, angle):
        self.current_angle = max(-80.0, min(80.0, angle))
        self.commands += 1

    def drive(self, angle, speed):
        """Combined steer+speed with the same deadband as the real one."""
        angle = max(-80.0, min(80.0, angle))
        if abs(angle - self.current_angle) < 1.0 and abs(speed - self.current_speed) < 1:
            return False
        self.current_angle = angle
        self.current_speed = speed
        self.commands += 1
        return True

    def steer_smooth(self, angle):
        self.steer(angle)

    def steer_center(self):
        self.steer(0.0)

    def stop_and_close(self):
        self.stop()

    def wait_for_start(self):
        pass


class SimRobot:
    """
    Bicycle-model chassis on the mat.

    Heading is degrees clockwise from +Y like everything else, so a positive
    (right) steering command increases it. Position integrates at
    PHYSICS_STEP_S regardless of how coarsely the control loop ticks, so the
    dynamics do not depend on the controller's rate.
    """

    def __init__(self, field_map, x, y, heading, wheelbase_mm=165.0,
                 max_road_wheel_deg=30.0, max_steer_command=80.0,
                 lag_s=STEER_LAG_S, rate_deg_s=STEER_RATE_DEG_S,
                 gain_error=STEER_GAIN_ERROR):
        self.map = field_map
        self.x, self.y, self.heading = float(x), float(y), float(heading)
        self.wheelbase_mm = wheelbase_mm
        # What the wheels ACTUALLY do at full lock, which is the config value
        # only when the config is right.
        self.max_road_wheel_deg = max_road_wheel_deg * gain_error
        self.max_steer_command = max_steer_command
        self.lag_s = float(lag_s)
        self.rate_deg_s = float(rate_deg_s)
        self.steer_actual = 0.0      # where the wheels are, not where asked

        self.distance_mm = 0.0
        self.min_clearance_mm = float("inf")
        self.crashed = False
        self.crash_point = None

    def advance(self, seconds, speed_command, steer_command):
        remaining = seconds
        while remaining > 1e-9:
            step = min(PHYSICS_STEP_S, remaining)
            remaining -= step
            self._integrate(step, speed_command, steer_command)
            self._check_clearance()

    def _track_steering(self, dt, steer_command):
        """
        Moves the wheels toward the commanded angle, rate-limited and lagged.
        Returns where they actually ended up.
        """
        target = steer_command
        if self.rate_deg_s > 0.0:
            step = self.rate_deg_s * dt
            target = self.steer_actual + max(-step, min(step, target - self.steer_actual))
        if self.lag_s > 0.0:
            alpha = 1.0 - math.exp(-dt / self.lag_s)
            self.steer_actual += alpha * (target - self.steer_actual)
        else:
            self.steer_actual = target
        return self.steer_actual

    def _integrate(self, dt, speed_command, steer_command):
        actual = self._track_steering(dt, steer_command)
        velocity = speed_command / 100.0 * TRUE_MM_PER_S_AT_FULL
        if velocity == 0.0:
            return
        road_wheel = math.radians(actual / self.max_steer_command
                                  * self.max_road_wheel_deg)

        radians = math.radians(self.heading)
        self.x += velocity * math.sin(radians) * dt
        self.y += velocity * math.cos(radians) * dt
        # + road wheel = turning right = heading increases, clockwise from +Y
        self.heading = (self.heading
                        + math.degrees(velocity / self.wheelbase_mm
                                       * math.tan(road_wheel) * dt)) % 360.0
        self.distance_mm += abs(velocity) * dt

    def _check_clearance(self):
        """
        Distance from the robot's edge to the nearest wall or block face.
        Negative means part of the chassis is through a wall.
        """
        to_outer = self.map.outer - max(abs(self.x), abs(self.y))
        to_block = max(abs(self.x), abs(self.y)) - self.map.inner
        clearance = min(to_outer, to_block) - ROBOT_RADIUS_MM

        if clearance < self.min_clearance_mm:
            self.min_clearance_mm = clearance
            if clearance <= 0.0 and not self.crashed:
                self.crashed = True
                self.crash_point = (self.x, self.y)

    @property
    def pose(self):
        return self.x, self.y, self.heading


class VirtualClock:
    """
    Replaces time.monotonic/time.sleep for the duration of a run.

    Every module in the project calls time.monotonic() through the `time`
    module rather than binding it at import, so patching the module's
    attributes redirects all of them at once - the task, the filter and the
    base runner all end up on the same virtual timeline.

    sleep() is where the world advances: the control loop's own pacing sleep
    is what drives the physics forward, so the robot moves exactly as far
    between ticks as the loop rate says it should.
    """

    def __init__(self, on_sleep=None):
        self.now = 10000.0
        self.on_sleep = on_sleep
        self._real_monotonic = time.monotonic
        self._real_sleep = time.sleep

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        seconds = max(0.0, float(seconds))
        self.now += seconds
        if self.on_sleep is not None:
            self.on_sleep(seconds)

    def __enter__(self):
        time.monotonic = self.monotonic
        time.sleep = self.sleep
        return self

    def __exit__(self, exc_type, exc, tb):
        time.monotonic = self._real_monotonic
        time.sleep = self._real_sleep
        return False


class SimContext:
    """A TaskContext stand-in holding the simulated hardware."""

    def __init__(self, field_map, robot, lidar, compass, debug=False, seed=0):
        self.debug = debug
        self.map = field_map
        self.robot = robot
        self.lidar = lidar
        self.compass = compass
        self.motor = SimMotor()
        # Seeded so a trial that fails can be re-run and watched. Without it
        # the particle filter's own randomness makes every run different.
        self.nav = NavigationManager(lidar, compass, field_map=field_map,
                                     debug=False, seed=seed)
        self.nav.start()
        # The obstacle round checks these before touching the camera.
        self.camera = None
        self.object_solver = None
        # No camera means no vision thread; the round falls back to its inline
        # detection path, which with no camera does nothing at all.
        self.vision = None

    def wait_for_start(self):
        pass

    def emergency_stop(self):
        self.motor.stop()


# ============================================================================
# Running a round
# ============================================================================

# How far a hand-placed robot realistically lands from the middle of its cell,
# and how far off square. The rules fix the cell and the two orientations; they
# do not fix millimeters, so the round has to tolerate this much slop.
PLACEMENT_JITTER_MM = 180.0
PLACEMENT_JITTER_DEG = 12.0


def random_placement(field_map, rng):
    """
    A legal starting spot: the middle of one of the four non-corner cells,
    facing one of that cell's two orientations, plus the slop of putting a
    robot down by hand. Corner cells are never used, per the rules.
    """
    poses = field_map.start_poses()
    for _ in range(50):
        _, x, y, heading = poses[rng.randrange(len(poses))]
        x += rng.uniform(-PLACEMENT_JITTER_MM, PLACEMENT_JITTER_MM)
        y += rng.uniform(-PLACEMENT_JITTER_MM, PLACEMENT_JITTER_MM)
        heading += rng.uniform(-PLACEMENT_JITTER_DEG, PLACEMENT_JITTER_DEG)
        if field_map.contains(x, y, ROBOT_RADIUS_MM + 40.0):
            return x, y, heading % 360.0
    raise AssertionError("no legal placement found - check the field dimensions")


def simulate(task_class, config, start, debug=False, window=False, timeout_s=180.0,
             seed=0, lag_s=STEER_LAG_S, rate_deg_s=STEER_RATE_DEG_S,
             gain_error=STEER_GAIN_ERROR):
    """
    Runs one round end to end. Returns a result dict; never raises for a
    driving failure, because "drove into the block" is a result, not an error.
    """
    field_map = FieldMap()
    robot = SimRobot(field_map, *start,
                     wheelbase_mm=config.get("pursuit.wheelbase_mm", 165.0),
                     max_road_wheel_deg=config.get("pursuit.max_road_wheel_deg", 30.0),
                     max_steer_command=config.get("pursuit.max_steer_command", 80.0),
                     lag_s=lag_s, rate_deg_s=rate_deg_s, gain_error=gain_error)
    lidar = SimLidar(field_map, seed=seed + 1)
    compass = SimCompass(drift_deg=0.0, seed=seed + 2)
    lidar.set_pose(*robot.pose)
    compass.set_true_heading(robot.heading)

    context = SimContext(field_map, robot, lidar, compass, debug=debug, seed=seed)
    task = task_class(context, config=config, max_runtime_s=timeout_s)

    offsets = []
    frames = []

    def advance(seconds):
        """Called from inside the round's own sleep - see VirtualClock."""
        robot.advance(seconds, context.motor.current_speed, context.motor.current_angle)
        lidar.set_pose(*robot.pose)
        compass.set_true_heading(robot.heading)
        if task.path is not None:
            _, lateral = task.path.project(robot.x, robot.y, task.direction)
            offsets.append(lateral)
        if window and len(frames) % 4 == 0:
            frames.append(True)

    with VirtualClock(on_sleep=advance) as clock:
        started = clock.now
        completed = task.run()
        elapsed = clock.now - started

    truth = Pose(robot.x, robot.y, robot.heading)
    return {
        "completed": completed,
        "crashed": robot.crashed,
        "crash_point": robot.crash_point,
        "min_clearance_mm": robot.min_clearance_mm,
        "laps": task.laps_done,
        "elapsed_s": elapsed,
        "driven_mm": robot.distance_mm,
        "direction": task.direction,
        "start": start,
        "final": truth,
        "max_offset_mm": max((abs(value) for value in offsets), default=0.0),
        "rms_offset_mm": float(np.sqrt(np.mean(np.square(offsets)))) if offsets else 0.0,
        "task": task,
        "context": context,
    }


def describe(result, index=None):
    label = "" if index is None else f"#{index:<3} "
    start = result["start"]
    verdict = ("CRASH" if result["crashed"]
               else "OK   " if result["completed"] else "SHORT")
    return (f"{label}{verdict}  start ({start[0] / 10:+6.1f},{start[1] / 10:+6.1f})cm "
            f"@{start[2]:5.1f}deg  {RacingLine.direction_name(result['direction'])[:4].upper()}  "
            f"laps {result['laps']:4.2f}  {result['elapsed_s']:5.1f}s  "
            f"clearance {result['min_clearance_mm'] / 10:+5.1f}cm  "
            f"off-line rms {result['rms_offset_mm'] / 10:4.1f}cm "
            f"max {result['max_offset_mm'] / 10:4.1f}cm")


def run_trials(task_class, config, count, seed, timeout_s):
    rng = random.Random(seed)
    field_map = FieldMap()
    results = []
    for index in range(count):
        start = random_placement(field_map, rng)
        result = simulate(task_class, config, start, timeout_s=timeout_s, seed=seed + index)
        results.append(result)
        print(describe(result, index))

    passed = [r for r in results if r["completed"] and not r["crashed"]]
    clearances = [r["min_clearance_mm"] for r in results]
    print(f"\n{len(passed)}/{len(results)} completed {config.get('laps.goal', 3)} laps "
          f"without touching anything")
    print(f"  worst clearance   {min(clearances) / 10:+.1f}cm")
    print(f"  median lap time   "
          f"{np.median([r['elapsed_s'] / max(r['laps'], 1e-6) for r in results]):.1f}s")
    print(f"  off-line rms      "
          f"{np.median([r['rms_offset_mm'] for r in results]) / 10:.1f}cm")
    for result in results:
        if not (result["completed"] and not result["crashed"]):
            print(f"  FAILED: {describe(result)}")
    return len(passed) == len(results)


def show_window(result):
    """Draws the finished run's field view, path and final pose."""
    import cv2

    task, context = result["task"], result["context"]
    canvas = context.nav.render_debug(truth=result["final"].as_tuple())
    task._draw_overlay(canvas)
    cv2.imshow("Driving", canvas)
    print("Final frame shown. Press any key with the window focused to close.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Closed-loop test of a whole round")
    parser.add_argument("--round", default="qualification",
                        choices=("qualification", "final"),
                        help="which round's task and config.toml to run")
    parser.add_argument("--start", type=float, nargs=3, default=None,
                        metavar=("X_MM", "Y_MM", "HEADING_DEG"),
                        help="where to place the robot (default: random)")
    parser.add_argument("--trials", type=int, default=1,
                        help="run N random placements and summarize")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--speed", type=int, default=None, help="override speed.base")
    parser.add_argument("--laps", type=int, default=None, help="override laps.goal")
    parser.add_argument("--config", default=None, help="alternative tunables TOML")
    parser.add_argument("--timeout", type=float, default=180.0,
                        help="virtual seconds before the round gives up")
    parser.add_argument("--window", action="store_true",
                        help="show the field view when the run finishes")
    args = parser.parse_args()

    if args.round == "final":
        from tasks.final.task import FinalTask as TaskClass
    else:
        from tasks.qualification.task import QualificationTask as TaskClass

    # Reuse the round's real config loading, CLI overrides and all.
    args.corner_speed = None
    config = load_config(TaskClass, args)

    if args.trials > 1:
        ok = run_trials(TaskClass, config, args.trials, args.seed, args.timeout)
        raise SystemExit(0 if ok else 1)

    placement = tuple(args.start) if args.start else random_placement(
        FieldMap(), random.Random(args.seed))
    outcome = simulate(TaskClass, config, placement, window=args.window,
                       timeout_s=args.timeout, seed=args.seed)
    print()
    print(describe(outcome))
    if args.window:
        show_window(outcome)
    raise SystemExit(0 if outcome["completed"] and not outcome["crashed"] else 1)

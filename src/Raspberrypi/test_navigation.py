"""
Manual test for classes/navigation_manager.py - lidar + IMU localization on
the 3.2m x 3.2m field with the 1m block in the middle.

Modes:
    python test_navigation.py                 simulator, OpenCV window, ESC quits
    python test_navigation.py --ascii         simulator, text only (plain SSH)
    python test_navigation.py --hardware      the real lidar and compass
    python test_navigation.py --hardware --ascii

Simulator mode needs no hardware at all: a fake robot drives a lap around the
ring, its lidar returns are raycast from the map and dosed with noise and
dropouts, and the filter has to find it. Ground truth is drawn in yellow, the
estimate in blue, so any error is visible directly - and printed in cm.

Useful flags:
    --lost              start the filter with no idea where the robot is, to
                         watch global localization converge (this is the real
                         startup case unless you seed a pose)
    --no-compass        run the simulator heading-blind. The map is 90-degree
                         symmetric, so watch it settle into a plausible but
                         possibly wrong quadrant - this is what the IMU is for.
    --imu-drift 15      degrees of IMU error to inject, to prove the wall-angle
                         correction absorbs it
    --fov -120 120      bearing window the lidar is trusted over (the default
                         cuts the arc where the beam hits the robot's body)
    --particles 500     particle count
    --speed 400         simulated drive speed, mm/s
    --duration 30       stop after N seconds

Hardware mode drives the real LidarManager and CompassManager. Put the robot on
the field, tell it where it is, and drive it around by hand:

    python test_navigation.py --hardware --start 1100 -1400 90

Add --camera to also map the red/green blocks. Each detection is placed in
field coordinates through the current pose and merged into nav.blocks, so the
blocks stay on the map after the camera has looked away - they are drawn as
life-size 5cm squares, hollow and grey until seen enough times to be confirmed:

    python test_navigation.py --hardware --camera

The lens defaults to 17cm straight back from the lidar, so --camera-offset is
only needed if it is mounted somewhere else. Both offsets are (forward, right)
from the REAR AXLE - the point the pose describes - and --lidar-offset moves
the camera with it. The lidar's default is the measured 15cm ahead of the
axle from classes/robot_geometry.py.

The blocks are only ever placed with a reliable pose, so if localization has
not converged you will see detections counted as rejected instead of mapped.
"""
import argparse
import math
import time

import cv2
import numpy as np

from classes.field_map import FieldMap
from classes.navigation_manager import NavigationManager
from classes.robot_geometry import (CAMERA_BEHIND_LIDAR_MM, DEFAULT_LIDAR_OFFSET_MM,
                                    to_field)
from utils.angle_utils import normalize_angle

# ============================================================================
# Simulator configuration
# ============================================================================
# The driven ring, midway between the center block and the outer wall. The
# corner radius has to leave the arcs clear of the block: they pull in to
# (PATH_HALF_MM - PATH_CORNER_MM), which must stay outside the block face plus
# the filter's ROBOT_CLEARANCE_MM, or the true pose is somewhere the particle
# filter is not allowed to put a particle.
PATH_HALF_MM = 1100.0
PATH_CORNER_MM = 400.0       # arcs pull in to 700mm, block face is at 600mm
SIM_SPEED_MM_S = 400.0
SIM_RANGE_NOISE_MM = 18.0    # the C1 is good to about a centimeter
SIM_DROPOUT = 0.20           # fraction of bearings with no return this tick
SIM_STEP_S = 0.1             # 10Hz, one C1 rotation


class SimPath:
    """
    A rounded square lap around the ring, parameterized by distance along it.

    Straights and quarter-circle corners, walked by arc length, which gives a
    continuous heading - a robot that teleports its heading at each corner
    would make the filter look worse than it is.
    """

    def __init__(self, half_mm=PATH_HALF_MM, corner_mm=PATH_CORNER_MM):
        half, corner = float(half_mm), float(corner_mm)
        straight = 2.0 * (half - corner)
        arc = math.pi / 2.0 * corner
        inner = half - corner

        # heading is degrees clockwise from +Y, so the direction vector for
        # heading h is (sin h, cos h) - same convention as everything else.
        self.segments = [
            {"line": ((0.0, -half), 90.0), "length": straight / 2},
            {"arc": ((inner, -inner), corner, -90.0), "length": arc},
            {"line": ((half, -inner), 0.0), "length": straight},
            {"arc": ((inner, inner), corner, 0.0), "length": arc},
            {"line": ((inner, half), 270.0), "length": straight},
            {"arc": ((-inner, inner), corner, 90.0), "length": arc},
            {"line": ((-half, inner), 180.0), "length": straight},
            {"arc": ((-inner, -inner), corner, 180.0), "length": arc},
            {"line": ((-inner, -half), 90.0), "length": straight / 2},
        ]
        self.length = sum(segment["length"] for segment in self.segments)

    def pose_at(self, distance_mm):
        """(x_mm, y_mm, heading_deg) that far along the lap."""
        remaining = distance_mm % self.length
        for segment in self.segments:
            if remaining > segment["length"]:
                remaining -= segment["length"]
                continue
            if "line" in segment:
                (start_x, start_y), heading = segment["line"]
                radians = math.radians(heading)
                return (start_x + remaining * math.sin(radians),
                        start_y + remaining * math.cos(radians), heading)
            (center_x, center_y), radius, start_angle = segment["arc"]
            angle = math.radians(start_angle + math.degrees(remaining / radius))
            # A point at math-angle `a` on the circle has tangent heading -a in
            # the clockwise-from-+Y convention (a=-90 at the bottom-right
            # corner entry gives heading 90, i.e. still driving +X).
            return (center_x + radius * math.cos(angle),
                    center_y + radius * math.sin(angle),
                    normalize_angle(-math.degrees(angle)))
        raise AssertionError("distance past the end of the path")


class SimLidar:
    """
    A LidarManager stand-in: raycasts the map from the true pose and adds the
    noise and dropouts a real C1 scan has. Only get_scan() is needed, which is
    all NavigationManager ever calls.

    `set_pose` takes the ROBOT's pose - the rear axle, like every pose in the
    stack - and the beam starts `offset_mm` ahead of it, where the real lidar
    is. So a sim that feeds this to a NavigationManager built with the same
    offset is testing the transform the real robot relies on, not skipping it.
    """

    def __init__(self, field_map, noise_mm=SIM_RANGE_NOISE_MM, dropout=SIM_DROPOUT,
                 body_radius_mm=110.0, seed=1, offset_mm=DEFAULT_LIDAR_OFFSET_MM):
        self.map = field_map
        self.noise_mm = noise_mm
        self.dropout = dropout
        self.body_radius_mm = body_radius_mm
        self.offset_mm = tuple(float(v) for v in offset_mm)
        self.pose = (0.0, 0.0, 0.0)
        self._rng = np.random.default_rng(seed)

    def set_pose(self, x_mm, y_mm, heading_deg):
        """The robot's (rear axle) pose; the beam origin is derived from it."""
        self.pose = (x_mm, y_mm, heading_deg)

    @property
    def origin(self):
        """(x, y, heading) of the beam itself, in field mm."""
        x, y, heading = self.pose
        origin_x, origin_y = to_field(x, y, heading, self.offset_mm)
        return float(origin_x), float(origin_y), heading

    def get_min_distance(self, start_deg, end_deg, max_age_s=None):
        """
        Closest return in a sector, sweeping clockwise from start to end.
        Same contract as LidarManager's - PathDrivingTask's front-clearance
        check calls this, so the stand-in has to answer it too.
        """
        scan = self.get_scan()
        start, end = int(math.floor(start_deg)) % 360, int(math.ceil(end_deg)) % 360
        indices = (np.arange(start, end + 1) if start <= end
                   else np.concatenate([np.arange(start, 360), np.arange(0, end + 1)]))
        sector = scan[indices % 360]
        if np.all(np.isnan(sector)):
            return np.nan, None
        closest = int(np.nanargmin(sector))
        return float(sector[closest]), int(indices[closest] % 360)

    def get_scan(self, max_age_s=None):
        x, y, heading = self.origin
        bearings = np.arange(360.0)
        ranges = self.map.raycast(x, y, np.radians(heading + bearings))
        ranges = ranges + self._rng.normal(0.0, self.noise_mm, 360)

        # The rear arc is blocked by the robot's own chassis, which is exactly
        # why NavigationManager crops the FOV - the sim has to reproduce it or
        # the crop looks like it costs nothing.
        blocked = ((bearings - 120.0) % 360.0) < 120.0
        ranges[blocked] = self.body_radius_mm + self._rng.normal(0.0, 5.0, np.count_nonzero(blocked))

        ranges[self._rng.random(360) < self.dropout] = np.nan
        return ranges


class SimCompass:
    """A CompassManager stand-in: true heading plus a fixed drift."""

    def __init__(self, drift_deg=0.0, noise_deg=1.0, seed=2):
        self.drift_deg = drift_deg
        self.noise_deg = noise_deg
        self.heading_deg = 0.0
        self._rng = np.random.default_rng(seed)

    def set_true_heading(self, heading_deg):
        self.heading_deg = heading_deg

    def heading(self):
        return normalize_angle(self.heading_deg + self.drift_deg
                               + self._rng.normal(0.0, self.noise_deg))

    def set_initial_heading(self):
        """Part of CompassManager's contract - base_task.run() calls it."""
        self.initial_heading = self.heading()
        self.target_heading = self.initial_heading
        return self.initial_heading

    def update(self):
        self.offset = 0.0
        return self.offset


# ============================================================================
# Runners
# ============================================================================

def build_sim(args):
    field_map = FieldMap()
    path = SimPath()
    lidar = SimLidar(field_map)
    compass = None if args.no_compass else SimCompass(drift_deg=args.imu_drift)

    nav = NavigationManager(lidar, compass, field_map=field_map, debug=True,
                            particle_count=args.particles,
                            fov_deg=tuple(args.fov) if args.fov else None,
                            seed=args.seed)

    truth = path.pose_at(0.0)
    lidar.set_pose(*truth)
    if compass is not None:
        compass.set_true_heading(truth[2])

    # The compass has to be anchored against the field before the filter can
    # use it - on the real robot this is "the robot starts facing +Y".
    if compass is not None:
        nav.set_start_heading(truth[2])
    if args.lost:
        nav.start()
    else:
        nav.start(truth[0], truth[1], truth[2])
    return nav, path, lidar, compass, truth


def sim_step(nav, path, lidar, compass, distance_mm, dt):
    """Advances the simulated robot and runs one filter update on its scan."""
    truth = path.pose_at(distance_mm)
    lidar.set_pose(*truth)
    if compass is not None:
        compass.set_true_heading(truth[2])
    nav.report_motion(SIM_SPEED_MM_S * dt)
    nav.update()
    return truth


def run_sim(args):
    nav, path, lidar, compass, truth = build_sim(args)
    print(f"Simulating a lap ({path.length / 1000:.2f}m) at {args.speed}mm/s. "
          f"{'Global localization' if args.lost else 'Seeded at the true pose'}.")
    if not args.ascii:
        print("Yellow = truth, blue = estimate, green = the scan the filter sees. "
              "ESC quits.")

    distance = 0.0
    errors = []
    started = time.monotonic()
    while True:
        distance += args.speed * SIM_STEP_S
        truth = sim_step(nav, path, lidar, compass, distance, SIM_STEP_S)

        pose = nav.get_pose(max_age_s=None)
        errors.append(math.hypot(truth[0] - pose.x, truth[1] - pose.y))

        if args.ascii:
            print("\033[2J\033[H" + nav.debug_text(truth), flush=True)
            time.sleep(SIM_STEP_S)
        else:
            try:
                cv2.imshow("Navigation", nav.render_debug(truth=truth))
            except cv2.error as e:
                print(f"Cannot open a window ({e.err.strip()}). Re-run with --ascii.")
                break
            if cv2.waitKey(int(SIM_STEP_S * 1000)) == 27:
                break

        if args.duration and time.monotonic() - started > args.duration:
            break

    cv2.destroyAllWindows()
    report_sim(errors)


def report_sim(errors):
    if not errors:
        return
    errors = np.asarray(errors)
    # The first second or so is the filter converging, which is not the number
    # you care about when the robot is already driving.
    settled = errors[min(len(errors) - 1, 20):]
    print(f"\n{len(errors)} updates")
    print(f"  position error: median {np.median(errors) / 10:.1f}cm  "
          f"p95 {np.percentile(errors, 95) / 10:.1f}cm  max {errors.max() / 10:.1f}cm")
    print(f"  once settled:   median {np.median(settled) / 10:.1f}cm  "
          f"p95 {np.percentile(settled, 95) / 10:.1f}cm  max {settled.max() / 10:.1f}cm")


def start_camera(args, nav):
    """
    Brings the camera and ObjectSolver up for block mapping. Returns
    (camera, solver), or (None, None) if the camera is off or unavailable -
    localization does not need it, so a missing camera is never fatal here.
    """
    if not args.camera:
        return None, None
    try:
        from camera_manager import CameraManager
        from classes.object_solver import ObjectSolver

        camera = CameraManager()
        camera.start_camera()
        solver = ObjectSolver(debug=False)
        # Left alone, nav already put the lens behind the lidar; only override
        # that if the mounting was measured and passed in.
        if args.camera_offset is not None:
            nav.blocks.camera_offset_mm = tuple(args.camera_offset)
        nav.blocks.debug = True
        print(f"Camera up, mapping blocks "
              f"(lens at {nav.blocks.camera_offset_mm}mm from the rear axle).")
        return camera, solver
    except Exception as e:
        print(f"WARNING: camera unavailable ({e!r}) - continuing without block mapping")
        return None, None


def observe_blocks(camera, solver, nav):
    """One camera frame, folded into nav's block map. Never raises."""
    if camera is None or solver is None:
        return
    try:
        camera.capture_image()
        camera.transform_image()
        nav.observe_blocks(solver.detect(camera.hsv_image))
    except Exception as e:
        print(f"WARNING: detection failed: {e!r}")


def run_hardware(args):
    from classes.compass_manager import CompassManager
    from classes.lidar_manager import LidarManager

    lidar = LidarManager(port=args.port, debug=True,
                         angle_offset_deg=args.offset, mirror=args.mirror)
    compass = CompassManager(debug=True)
    compass.start()

    nav = NavigationManager(lidar, compass if compass.is_available else None,
                            debug=True, particle_count=args.particles,
                            fov_deg=tuple(args.fov) if args.fov else None,
                            compass_sign=args.compass_sign,
                            lidar_offset_mm=tuple(args.lidar_offset))
    camera, solver = start_camera(args, nav)

    with lidar:
        if not lidar.wait_until_ready(timeout=5.0):
            print("Warning: little or no lidar data - is the motor spinning?")

        if args.start:
            x, y, heading = args.start
            nav.start(x, y, heading)
            print(f"Seeded at ({x}, {y})mm facing {heading}deg.")
        else:
            nav.start()
            print("No --start given, searching the whole field. "
                  "Expect a second or two, and a coin-toss quadrant if the IMU is dead.")

        print("Push the robot around by hand and watch the pose. Ctrl-C / ESC quits.")
        deadline = time.monotonic() + args.duration if args.duration else None
        tick = 0
        try:
            while lidar.is_running:
                nav.update()
                # The camera pipeline costs far more than a filter tick, so it
                # runs on its own slower cadence - same trick FinalTask uses.
                if tick % args.camera_every == 0:
                    observe_blocks(camera, solver, nav)
                tick += 1
                if args.ascii:
                    print("\033[2J\033[H" + nav.debug_text(), flush=True)
                    time.sleep(0.2)
                else:
                    try:
                        cv2.imshow("Navigation", nav.render_debug())
                    except cv2.error as e:
                        print(f"Cannot open a window ({e.err.strip()}). Use --ascii.")
                        break
                    if cv2.waitKey(30) == 27:
                        break
                if deadline and time.monotonic() > deadline:
                    break
        except KeyboardInterrupt:
            print()

    cv2.destroyAllWindows()
    if camera is not None:
        try:
            camera.release_video()
        except Exception:
            pass
    print(nav.status_line())
    print(nav.blocks.debug_text())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test lidar + IMU field localization")
    parser.add_argument("--hardware", action="store_true",
                        help="use the real lidar and compass instead of the simulator")
    parser.add_argument("--ascii", action="store_true",
                        help="print the readout instead of opening a window")
    parser.add_argument("--particles", type=int, default=500)
    parser.add_argument("--fov", type=float, nargs=2, default=[-120.0, 120.0],
                        metavar=("START", "END"),
                        help="bearings the lidar is trusted over (default: -120 120, "
                             "cutting the arc blocked by the robot's body)")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="stop after N seconds (0 = run until quit)")

    simulation = parser.add_argument_group("simulator")
    simulation.add_argument("--lost", action="store_true",
                            help="do not seed the filter with the true pose")
    simulation.add_argument("--no-compass", action="store_true",
                            help="run heading-blind, to show the 90-degree ambiguity")
    simulation.add_argument("--imu-drift", type=float, default=0.0,
                            help="degrees of constant IMU error to inject")
    simulation.add_argument("--speed", type=float, default=SIM_SPEED_MM_S,
                            help="simulated drive speed in mm/s")
    simulation.add_argument("--seed", type=int, default=0)

    hardware = parser.add_argument_group("hardware")
    hardware.add_argument("--port", default=None, help="lidar port (default: autodetect)")
    hardware.add_argument("--offset", type=float, default=0.0,
                          help="degrees to rotate the lidar's zero onto robot forward")
    hardware.add_argument("--mirror", action="store_true",
                          help="lidar is mounted upside down")
    hardware.add_argument("--start", type=float, nargs=3, default=None,
                          metavar=("X_MM", "Y_MM", "HEADING_DEG"),
                          help="where the robot really is, to seed the filter")
    hardware.add_argument("--compass-sign", type=float, default=1.0,
                          help="+1 if the IMU counts up clockwise, -1 otherwise")
    hardware.add_argument("--lidar-offset", type=float, nargs=2,
                          default=list(DEFAULT_LIDAR_OFFSET_MM),
                          metavar=("FORWARD_MM", "RIGHT_MM"),
                          help=f"where the lidar sits relative to the rear axle "
                               f"(default: {DEFAULT_LIDAR_OFFSET_MM[0]:.0f}mm ahead)")
    hardware.add_argument("--camera", action="store_true",
                          help="also map the red/green blocks with the camera")
    hardware.add_argument("--camera-offset", type=float, nargs=2, default=None,
                          metavar=("FORWARD_MM", "RIGHT_MM"),
                          help=f"where the lens sits relative to the rear axle "
                               f"(default: {CAMERA_BEHIND_LIDAR_MM:.0f}mm behind the lidar)")
    hardware.add_argument("--camera-every", type=int, default=5,
                          help="run detection every N filter ticks (default: 5)")
    args = parser.parse_args()

    if args.hardware:
        run_hardware(args)
    else:
        run_sim(args)

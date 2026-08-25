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
    python test_driving.py --round final      ... with four pillars to dodge
    python test_driving.py --round final --pillars 6
    python test_driving.py --round final --trials 8 --sweep blocks.approach_mm 400,800,1200

What it checks, per run:
    * did it complete the laps, and how long did it take
    * how close did it get to a wall or to the centre block (the number that
      matters - anything at or below zero is a crash)
    * how far off the racing line did it drift
    * every pillar: which side it was passed on, and by how much

The pillars are seen through a camera that gets things WRONG the way the real
one does - range error growing with the square of distance, bearing noise, and
dropouts that get likelier the further away a pillar is. Everything downstream
of that is real: nav.observe_blocks, the whole of BlockMap with its
confirmation and pruning, and the round's own steering. A simulator that
handed the round a perfect map could not reproduce a single one of the
failures that actually happen on the mat, all of which start with a detection
that was late, wrong or missing.

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

from classes.block_map import (BLOCK_SIZE_MM, MAX_MAPPING_RANGE_MM,
                               camera_offset_behind_lidar)
from classes.field_map import FieldMap
from classes.navigation_manager import NavigationManager, Pose
from classes.object_solver import CAPTURED_HORIZONTAL_FOV_DEG, DetectedObject
from classes.parking import (NOMINAL_BAY_MM, WALL_THICKNESS_MM, along_axis_of,
                             bay_interior, section_of, wall_heading_of,
                             wall_rects)
from classes.racing_line import RacingLine
from tasks.cli import load_config
from utils.angle_utils import angle_difference
from utils.enums import Color
from test_navigation import SimCompass, SimLidar

# ============================================================================
# Simulated robot
# ============================================================================
ROBOT_RADIUS_MM = 100.0      # placement sanity only - see body_corners for the
                             # real chassis, which is a rectangle, not a disc
PHYSICS_STEP_S = 0.02        # bicycle model integration step

# The chassis as a rectangle. A disc was fine while the only things to miss
# were the outer wall and the centre block, both of which the robot passes
# broadside; the parking bay is 16mm-per-end tight at the worst angle, and a
# 100mm disc simply cannot express that. Measured from the POSE POINT, which
# in this sim is the rear axle (the bicycle model integrates about it and
# pursuit.rear_axle_offset_mm is 0).
ROBOT_LENGTH_MM = 240.0
ROBOT_WIDTH_MM = 120.0
ROBOT_REAR_OVERHANG_MM = 40.0    # rear axle back to the rear bumper
# Furthest corner from the pose point, for cheap "is this box even near us"
# rejects before the real polygon test.
BODY_RADIUS_MM = math.hypot(ROBOT_LENGTH_MM - ROBOT_REAR_OVERHANG_MM,
                            ROBOT_WIDTH_MM / 2.0)

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


def body_corners(x, y, heading_deg, length_mm=ROBOT_LENGTH_MM,
                 width_mm=ROBOT_WIDTH_MM, rear_overhang_mm=ROBOT_REAR_OVERHANG_MM):
    """
    The four corners of the chassis, in field mm, for a pose whose reference
    point is the rear axle. Heading is degrees clockwise from +Y, so forward is
    (sin, cos) and right is (cos, -sin).
    """
    radians = math.radians(heading_deg)
    forward = (math.sin(radians), math.cos(radians))
    right = (math.cos(radians), -math.sin(radians))
    back, front = -rear_overhang_mm, length_mm - rear_overhang_mm
    half = width_mm / 2.0
    return [(x + along * forward[0] + across * right[0],
             y + along * forward[1] + across * right[1])
            for along, across in ((front, -half), (front, half),
                                  (back, half), (back, -half))]


def _separation_mm(poly_a, poly_b):
    """
    How far apart two convex polygons are, by the separating axis theorem:
    positive is a real gap, negative means they overlap by that much.

    Exact for the disjoint case along the best axis, which is all the clearance
    report needs, and exact as a yes/no for contact - which is what decides a
    crash.
    """
    best = -float("inf")
    for poly in (poly_a, poly_b):
        count = len(poly)
        for index in range(count):
            x0, y0 = poly[index]
            x1, y1 = poly[(index + 1) % count]
            axis = (-(y1 - y0), x1 - x0)
            length = math.hypot(*axis)
            if length < 1e-9:
                continue
            axis = (axis[0] / length, axis[1] / length)
            a = [px * axis[0] + py * axis[1] for px, py in poly_a]
            b = [px * axis[0] + py * axis[1] for px, py in poly_b]
            best = max(best, max(min(b) - max(a), min(a) - max(b)))
    return best


def _rect_corners(low, high):
    """An axis-aligned (low, high) box as a corner list, for _separation_mm."""
    return [(low[0], low[1]), (high[0], low[1]),
            (high[0], high[1]), (low[0], high[1])]


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
        # The simulated chassis always drives; the flag exists because
        # PathDrivingTask asks the motor whether it does (see --no-drive).
        self.drive_enabled = True

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
        # Kept apart on purpose: the bay is a place the robot is SUPPOSED to
        # squeeze into, so folding its 16mm-per-end slack into the same number
        # as "how close did we come to the centre block" would make every
        # successful park look like a near miss.
        self.min_clearance_mm = float("inf")     # outer wall and centre block
        self.min_bay_clearance_mm = float("inf")  # the parking walls
        self.crashed = False
        self.hit_bay = False
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
        Distance from the robot's EDGE to the nearest wall, block face or
        parking wall. Negative means part of the chassis is through something.

        The chassis is a rotated rectangle, not a disc: at 26.6 degrees it is
        268mm along the wall against a 300mm bay, and a disc has no way to say
        that.
        """
        corners = body_corners(self.x, self.y, self.heading)

        # Outer wall: the robot is INSIDE it, so the gap is whatever room the
        # furthest-out corner has left.
        clearance = self.map.outer - max(max(abs(cx), abs(cy)) for cx, cy in corners)
        # Centre block: the robot is OUTSIDE it, so it is a polygon gap.
        clearance = min(clearance, _separation_mm(
            corners, _rect_corners((-self.map.inner, -self.map.inner),
                                   (self.map.inner, self.map.inner))))

        if clearance < self.min_clearance_mm:
            self.min_clearance_mm = clearance
            if clearance <= 0.0 and not self.crashed:
                self.crashed = True
                self.crash_point = (self.x, self.y)

        for low, high in self.map.obstacles:
            # Cheap circle reject first - SAT runs every physics step against
            # every box, and most steps are nowhere near the bay.
            box_x, box_y = (low[0] + high[0]) / 2.0, (low[1] + high[1]) / 2.0
            box_radius = math.hypot(high[0] - low[0], high[1] - low[1]) / 2.0
            if math.hypot(box_x - self.x, box_y - self.y) > box_radius + BODY_RADIUS_MM:
                continue
            gap = _separation_mm(corners, _rect_corners(low, high))
            self.min_bay_clearance_mm = min(self.min_bay_clearance_mm, gap)
            if gap <= 0.0 and not self.hit_bay:
                self.hit_bay = True
                self.crashed = True
                self.crash_point = self.crash_point or (self.x, self.y)

    @property
    def pose(self):
        return self.x, self.y, self.heading


# ============================================================================
# Simulated pillars, and a camera that sees them the way the real one does
# ============================================================================
# How far off the racing line a pillar may be dropped. The corridor is 1000mm
# wide, so this keeps them inside it with room for the robot to get past on
# either side - which is the interesting case, because a pillar sitting wide
# needs a smaller dodge than one on the line and the round has to work that
# out rather than applying a fixed offset.
PILLAR_LATERAL_MM = 220.0

# Two pillars closer together than this cannot be dodged as separate things -
# the robot would still be crossing to one side of the first as the second
# demanded the other. The rules space them out; so does this.
PILLAR_MIN_SPACING_MM = 800.0

# What the camera gets WRONG, which is the whole reason for simulating it
# rather than handing the round a perfect map. ObjectSolver ranges a pillar
# off its apparent height, so a pixel of contour noise is worth more the
# further away the pillar is - the error grows with the square of range. At
# 2m a 100mm box is a few pixels tall and the estimate is soft.
PILLAR_RANGE_ERROR_AT_1M = 0.02      # fraction of range, at 1m, growing as r^2
PILLAR_BEARING_ERROR_DEG = 1.0

# And what it MISSES. A marginal pillar is not missed cleanly once - it drops
# in and out frame to frame, which is what makes a dodge computed from live
# detections flicker. Probability of missing a given frame, at max range.
PILLAR_DROPOUT_AT_RANGE = 0.4


class SimPillar:
    """One traffic sign on the mat, and how the run went past it."""

    def __init__(self, x, y, color, progress):
        self.x, self.y, self.color, self.progress = x, y, color, progress
        self.min_clearance_mm = float("inf")   # body to pillar face
        self.side = None                       # + = pillar passed on our right

    @property
    def passed_correctly(self):
        """
        GREEN is passed on its LEFT, which puts it to the robot's RIGHT.

        None until the robot has actually been alongside it - a pillar never
        reached is not a pillar passed wrongly, and the two need telling apart
        in the summary.
        """
        if self.side is None:
            return None
        return self.side > 0.0 if self.color is Color.GREEN else self.side < 0.0

    def observe(self, x, y, heading):
        """
        Closest approach and which side we were on when we got there.

        Measured against ROBOT_RADIUS_MM - the sim's own idea of how wide the
        chassis is - not against the config's robot_half_width_mm, on purpose
        and for the same reason STEER_GAIN_ERROR exists: the sim holds the
        truth and the config holds a belief about it, and a run is worth
        nothing if it quietly assumes the two agree. A correctly executed
        dodge therefore reads clearance_mm - (ROBOT_RADIUS_MM -
        robot_half_width_mm), which is 18cm at the moment, NOT the 22cm
        clearance_mm asks for. If that gap bothers you, one of the two numbers
        is wrong about the actual robot - and finding that out here is the
        point.
        """
        offset_x, offset_y = self.x - x, self.y - y
        clearance = math.hypot(offset_x, offset_y) - BLOCK_SIZE_MM / 2.0 - ROBOT_RADIUS_MM
        if clearance >= self.min_clearance_mm:
            return
        self.min_clearance_mm = clearance
        radians = math.radians(heading)
        # + when the pillar is off the robot's right shoulder.
        self.side = offset_x * math.cos(radians) - offset_y * math.sin(radians)


class SimCamera:
    """
    Stands in for CameraManager. The "frame" it hands back is the robot's true
    pose, because the only thing downstream of it here is a solver that knows
    where the pillars really are - see SimObjectSolver.
    """

    def __init__(self, robot):
        self.robot = robot
        self.display_image = None

    def capture_for_blocks(self, with_display=False):
        return self.robot.pose


class SimObjectSolver:
    """
    Stands in for ObjectSolver: turns the true pillar positions into the
    detections the real one would have produced from a frame, complete with
    the range error, the bearing error and the dropouts.

    Everything downstream of this is real - nav.observe_blocks, the whole of
    BlockMap with its confirmation counting, miss counting and pruning, and
    the round's own steering. Only the pixels are invented, which is the point:
    the failures worth catching here are the ones that come from imperfect
    detection, and a simulator that hands the round a clean map cannot show
    any of them.
    """

    debug = False

    def __init__(self, pillars, field_map, rng, max_range_mm, fov_deg=None):
        self.pillars = pillars
        self.map = field_map
        self.rng = rng
        self.max_range_mm = float(max_range_mm)
        self.fov_deg = float(fov_deg or CAPTURED_HORIZONTAL_FOV_DEG)
        self.camera_offset_mm = camera_offset_behind_lidar()
        self.frames = 0
        self.reported = 0

    def show_debug(self):
        """No windows to paint - the round calls this when --debug is on."""

    def close_debug(self):
        """Ditto, on the way out."""

    def detect(self, frame, display_image=None):
        """
        I/O:
            frame: whatever capture_for_blocks returned - here, the true pose
            return: list of DetectedObject, camera-relative, in the same units
                    and conventions the real solver uses
        """
        x, y, heading = frame
        forward, right = self.camera_offset_mm
        radians = math.radians(heading)
        lens_x = x + forward * math.sin(radians) + right * math.cos(radians)
        lens_y = y + forward * math.cos(radians) - right * math.sin(radians)

        self.frames += 1
        detections = []
        for pillar in self.pillars:
            offset_x, offset_y = pillar.x - lens_x, pillar.y - lens_y
            distance = math.hypot(offset_x, offset_y)
            if not 1.0 < distance <= self.max_range_mm:
                continue
            bearing = angle_difference(math.degrees(math.atan2(offset_x, offset_y)),
                                       heading)
            if abs(bearing) > self.fov_deg / 2.0:
                continue
            # The centre block hides half the field for half of every lap.
            sightline = float(self.map.raycast(
                lens_x, lens_y, math.radians(heading + bearing)))
            if sightline < distance - BLOCK_SIZE_MM:
                continue
            # Further away is likelier to be missed entirely, and this frame's
            # miss is independent of the last one's - that flicker is the
            # thing worth simulating.
            if self.rng.random() < PILLAR_DROPOUT_AT_RANGE * (distance / self.max_range_mm) ** 2:
                continue

            metres = distance / 1000.0
            error = self.rng.gauss(0.0, PILLAR_RANGE_ERROR_AT_1M * metres ** 2 * distance)
            seen_at = max(BLOCK_SIZE_MM, distance + error)
            seen_bearing = bearing + self.rng.gauss(0.0, PILLAR_BEARING_ERROR_DEG)
            self.reported += 1
            detections.append(DetectedObject(
                color=pillar.color,
                # BlockMap adds the half-block back on, so this is the face.
                distance_cm=(seen_at - BLOCK_SIZE_MM / 2.0) / 10.0,
                bearing_deg=seen_bearing,
                forward_cm=seen_at * math.cos(math.radians(seen_bearing)) / 10.0,
                lateral_cm=seen_at * math.sin(math.radians(seen_bearing)) / 10.0,
                pixel_center=(0, 0),
                box_points=np.zeros((4, 2))))
        return detections


def place_pillars(path, direction, count, rng):
    """
    `count` pillars spread around the lap, alternating colour, each dropped
    somewhere in the corridor rather than exactly on the racing line.

    Placed by progress along the lap and offset sideways, because that is how
    they matter: what the round has to get right is the distance between one
    pillar and the next, and how far each sits off the line it is driving.
    """
    if count <= 0:
        return []
    spacing = path.length / count
    if spacing < PILLAR_MIN_SPACING_MM:
        raise SystemExit(f"{count} pillars on a {path.length / 1000:.1f}m lap would sit "
                         f"{spacing:.0f}mm apart; nothing can dodge that. Use fewer.")

    pillars = []
    for index in range(count):
        progress = (index + 0.5) * spacing + rng.uniform(-spacing / 6.0, spacing / 6.0)
        lateral = rng.uniform(-PILLAR_LATERAL_MM, PILLAR_LATERAL_MM)
        x, y = path.point_at(progress, direction, lateral)
        color = Color.GREEN if rng.random() < 0.5 else Color.RED
        pillars.append(SimPillar(x, y, color, progress))
    return pillars


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

    def __init__(self, field_map, robot, lidar, compass, debug=False, seed=0,
                 pillars=None, camera_range_mm=None, nav_map=None):
        self.debug = debug
        self.map = field_map
        self.robot = robot
        self.lidar = lidar
        self.compass = compass
        self.motor = SimMotor()
        # Seeded so a trial that fails can be re-run and watched. Without it
        # the particle filter's own randomness makes every run different.
        # Deliberately a DIFFERENT map object from the one the lidar raycasts
        # and the collision check uses, when there is a parking bay: the truth
        # map has the bay walls on it, and the robot is not supposed to know
        # where they are until it has found them. Sharing one map would hand
        # the particle filter the answer for free.
        self.nav = NavigationManager(lidar, compass,
                                     field_map=field_map if nav_map is None else nav_map,
                                     debug=False, seed=seed)
        self.nav.start()
        self.pillars = pillars or []
        if self.pillars:
            self.camera = SimCamera(robot)
            self.object_solver = SimObjectSolver(
                self.pillars, field_map, random.Random(seed + 3),
                max_range_mm=camera_range_mm or MAX_MAPPING_RANGE_MM)
        else:
            # The obstacle round checks these before touching the camera.
            self.camera = None
            self.object_solver = None
        # No vision THREAD either way: the round falls back to its inline
        # detection path, which runs on the control loop every
        # CAMERA_EVERY_N_TICKS ticks - the same rate, and deterministic, which
        # a thread would not be.
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


# The bay lives in the same cell the robot starts in, but never on top of it.
MIN_BAY_TO_START_MM = 450.0


def place_bay(start, field_map, rng):
    """
    A parking bay in the robot's own start section, clear of where the robot
    is standing.

    I/O:
        start: the (x, y, heading) the robot was placed at
        return: (section_name, centre_along_mm), or None if the placement was
                in a corner cell (which the rules do not allow anyway)
    """
    section = section_of(start[0], start[1], field_map)
    if section is None:
        return None
    axis = along_axis_of(section)
    # The whole bay, walls included, has to stay inside the 1000mm cell.
    reach = field_map.inner - (NOMINAL_BAY_MM / 2.0 + WALL_THICKNESS_MM)
    for _ in range(50):
        center = rng.uniform(-reach, reach)
        if abs(center - start[axis]) >= MIN_BAY_TO_START_MM:
            return section, center
    return section, math.copysign(reach, -start[axis])


def parked_state(robot, bay, field_map):
    """
    Is the robot parked, and how well.

    "Parked" is the whole chassis inside the box between the two walls - see
    parking.bay_interior. The heading is measured against the outer wall's own
    axis and folded into 0-90, because facing either way along the bay is
    equally parked.

    I/O:
        return: dict with parked, heading_error_deg, and how far the worst
                corner is outside the bay (0.0 when parked)
    """
    section, center = bay
    low, high = bay_interior(section, center, field_map)
    corners = body_corners(robot.x, robot.y, robot.heading)
    outside = max(max(low[0] - cx, cx - high[0], low[1] - cy, cy - high[1])
                  for cx, cy in corners)
    error = abs(angle_difference(robot.heading, wall_heading_of(section)))
    return {"parked": outside <= 0.0,
            "outside_mm": max(0.0, outside),
            "heading_error_deg": min(error, 180.0 - error)}


def simulate(task_class, config, start, debug=False, window=False, timeout_s=180.0,
             seed=0, lag_s=STEER_LAG_S, rate_deg_s=STEER_RATE_DEG_S,
             gain_error=STEER_GAIN_ERROR, pillars=0, bay=None):
    """
    Runs one round end to end. Returns a result dict; never raises for a
    driving failure, because "drove into the block" is a result, not an error.
    """
    field_map = FieldMap()
    # The bay goes on the TRUTH map only - what the lidar sees and what the
    # chassis can hit. context.nav gets a clean one; finding the walls is the
    # round's job. See SimContext.
    nav_map = None
    if bay is not None:
        field_map.set_obstacles(wall_rects(bay[0], bay[1], field_map))
        nav_map = FieldMap()
    robot = SimRobot(field_map, *start,
                     wheelbase_mm=config.get("pursuit.wheelbase_mm", 165.0),
                     max_road_wheel_deg=config.get("pursuit.max_road_wheel_deg", 30.0),
                     max_steer_command=config.get("pursuit.max_steer_command", 80.0),
                     lag_s=lag_s, rate_deg_s=rate_deg_s, gain_error=gain_error)
    lidar = SimLidar(field_map, seed=seed + 1)
    compass = SimCompass(drift_deg=0.0, seed=seed + 2)
    lidar.set_pose(*robot.pose)
    compass.set_true_heading(robot.heading)

    # Placed on the same loop the round will drive, so "700mm apart" means
    # 700mm of LAP between them, which is the distance that decides whether
    # two dodges have room to happen one after the other. Direction is +1
    # here regardless of which way the robot ends up going: it only picks
    # which end of the loop progress counts from, and a pillar is in the same
    # place either way.
    placed = place_pillars(
        RacingLine(field_map=field_map,
                   wall_margin_mm=config.get("path.wall_margin_mm"),
                   corner_radius_mm=config.get("path.corner_radius_mm"),
                   resolution_mm=config.get("path.resolution_mm", 20.0)),
        1, pillars, random.Random(seed + 4))

    context = SimContext(field_map, robot, lidar, compass, debug=debug, seed=seed,
                         pillars=placed, nav_map=nav_map,
                         camera_range_mm=config.get("blocks.map_range_mm"))
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
        for pillar in placed:
            pillar.observe(robot.x, robot.y, robot.heading)
        if window and len(frames) % 4 == 0:
            frames.append(True)

    with VirtualClock(on_sleep=advance) as clock:
        started = clock.now
        completed = task.run()
        elapsed = clock.now - started

    truth = Pose(robot.x, robot.y, robot.heading)
    parking = (parked_state(robot, bay, field_map) if bay is not None else None)
    return {
        "bay": bay,
        "parking": parking,
        "hit_bay": robot.hit_bay,
        "bay_clearance_mm": (None if robot.min_bay_clearance_mm == float("inf")
                             else robot.min_bay_clearance_mm),
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
        "pillars": placed,
        "pillars_wrong": [p for p in placed if p.passed_correctly is False],
        "pillars_hit": [p for p in placed if p.min_clearance_mm <= 0.0],
        "pillar_clearance_mm": min((p.min_clearance_mm for p in placed), default=None),
        "detections": (context.object_solver.reported if context.object_solver else 0),
        "frames": (context.object_solver.frames if context.object_solver else 0),
        "rms_offset_mm": float(np.sqrt(np.mean(np.square(offsets)))) if offsets else 0.0,
        "task": task,
        "context": context,
    }


def describe(result, index=None):
    label = "" if index is None else f"#{index:<3} "
    start = result["start"]
    verdict = ("BAYHIT" if result["hit_bay"]
               else "CRASH" if result["crashed"]
               else "WRONG" if result["pillars_wrong"] or result["pillars_hit"]
               else "PARKED" if result["parking"] and result["parking"]["parked"]
               else "NOPARK" if result["parking"]
               else "OK   " if result["completed"] else "SHORT")
    line = (f"{label}{verdict}  start ({start[0] / 10:+6.1f},{start[1] / 10:+6.1f})cm "
            f"@{start[2]:5.1f}deg  {RacingLine.direction_name(result['direction'])[:4].upper()}  "
            f"laps {result['laps']:4.2f}  {result['elapsed_s']:5.1f}s  "
            f"clearance {result['min_clearance_mm'] / 10:+5.1f}cm  "
            f"off-line rms {result['rms_offset_mm'] / 10:4.1f}cm "
            f"max {result['max_offset_mm'] / 10:4.1f}cm")
    if result["pillars"]:
        right = sum(1 for p in result["pillars"] if p.passed_correctly)
        line += (f"  pillars {right}/{len(result['pillars'])} correct"
                 f" closest {result['pillar_clearance_mm'] / 10:+.1f}cm")
    if result["parking"]:
        gap = result["bay_clearance_mm"]
        line += (f"  bay {result['bay'][0]}@{result['bay'][1] / 10:+.0f}cm"
                 f" off-square {result['parking']['heading_error_deg']:4.1f}deg"
                 f" outside {result['parking']['outside_mm'] / 10:4.1f}cm"
                 f" closest {'n/a' if gap is None else f'{gap / 10:+.1f}cm'}")
    return line


def run_trials(task_class, config, count, seed, timeout_s, pillars=0, quiet=False,
               parking=False):
    rng = random.Random(seed)
    field_map = FieldMap()
    results = []
    for index in range(count):
        start = random_placement(field_map, rng)
        bay = place_bay(start, field_map, rng) if parking else None
        result = simulate(task_class, config, start, timeout_s=timeout_s,
                          seed=seed + index, pillars=pillars, bay=bay)
        results.append(result)
        if not quiet:
            print(describe(result, index))
    if quiet:
        return results

    passed = [r for r in results if r["completed"] and not r["crashed"]
              and not r["pillars_wrong"] and not r["pillars_hit"]
              and (r["parking"] is None or r["parking"]["parked"])]
    clearances = [r["min_clearance_mm"] for r in results]
    print(f"\n{len(passed)}/{len(results)} completed {config.get('laps.goal', 3)} laps "
          f"without touching anything")
    print(f"  worst clearance   {min(clearances) / 10:+.1f}cm")
    if any(r["pillars"] for r in results):
        print(f"  pillars           {_pillar_summary(results)}")
    if any(r["parking"] for r in results):
        parked = sum(1 for r in results if r["parking"]["parked"])
        bumped = sum(1 for r in results if r["hit_bay"])
        gaps = [r["bay_clearance_mm"] for r in results
                if r["bay_clearance_mm"] is not None]
        print(f"  parked            {parked}/{len(results)}, "
              f"{bumped} touched a bay wall"
              + (f", closest {min(gaps) / 10:+.1f}cm" if gaps else ""))
    print(f"  median lap time   "
          f"{np.median([r['elapsed_s'] / max(r['laps'], 1e-6) for r in results]):.1f}s")
    print(f"  off-line rms      "
          f"{np.median([r['rms_offset_mm'] for r in results]) / 10:.1f}cm")
    for result in results:
        if result not in passed:
            print(f"  FAILED: {describe(result)}")
    return len(passed) == len(results)


def _pillar_summary(results):
    """One line of how the pillars themselves went, across a set of runs."""
    every = [p for r in results for p in r["pillars"]]
    if not every:
        return "none placed"
    right = sum(1 for p in every if p.passed_correctly)
    wrong = sum(1 for p in every if p.passed_correctly is False)
    missed = sum(1 for p in every if p.passed_correctly is None)
    clearances = [p.min_clearance_mm for p in every if p.passed_correctly is not None]
    seen = sum(r["detections"] for r in results)
    frames = sum(r["frames"] for r in results)
    line = f"{right} passed correctly, {wrong} on the wrong side, {missed} never reached"
    if clearances:
        line += (f"; closest {min(clearances) / 10:+.1f}cm, "
                 f"median {np.median(clearances) / 10:+.1f}cm")
    return f"{line}; {seen} detections over {frames} frames"


def run_sweep(task_class, config, key, values, count, seed, timeout_s, pillars):
    """
    The same trials at several values of one setting, side by side.

    This is what makes a tuning knob checkable rather than arguable: if a
    column does not move when the value does, the setting is not reaching the
    behaviour - which is a different problem from it being set wrong, and
    needs a different fix.
    """
    print(f"Sweeping {key} over {', '.join(str(v) for v in values)} "
          f"({count} placements each, {pillars} pillars)\n")
    header = (f"{key.split('.')[-1]:>12}  {'passed':>8}  {'correct':>9}  "
              f"{'closest':>9}  {'median':>9}  {'off-line rms':>13}")
    print(header)
    print("-" * len(header))
    for value in values:
        config.set(key, value)
        results = run_trials(task_class, config, count, seed, timeout_s,
                             pillars=pillars, quiet=True)
        every = [p for r in results for p in r["pillars"]]
        judged = [p for p in every if p.passed_correctly is not None]
        clean = sum(1 for r in results if r["completed"] and not r["crashed"]
                    and not r["pillars_wrong"] and not r["pillars_hit"])
        right = sum(1 for p in every if p.passed_correctly)
        closest = min((p.min_clearance_mm for p in judged), default=float("nan"))
        median = np.median([p.min_clearance_mm for p in judged]) if judged else float("nan")
        rms = np.median([r["rms_offset_mm"] for r in results])
        print(f"{value:>12}  {clean:>4}/{len(results):<3}  {right:>4}/{len(every):<4}  "
              f"{closest / 10:>+8.1f}cm  {median / 10:>+8.1f}cm  {rms / 10:>12.1f}cm")


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
    parser.add_argument("--pillars", type=int, default=None, metavar="N",
                        help="traffic signs to place around the lap "
                             "(default: 4 for the final round, 0 otherwise)")
    parser.add_argument("--parking", action="store_true",
                        help="put a parking bay in the robot's start section and "
                             "require the round to end parked between its walls")
    parser.add_argument("--sweep", nargs=2, default=None, metavar=("KEY", "VALUES"),
                        help="run the trials once per value and compare, e.g. "
                             "--sweep blocks.approach_mm 400,800,1200,1600")
    args = parser.parse_args()

    if args.round == "final":
        from tasks.final.task import FinalTask as TaskClass
    else:
        from tasks.qualification.task import QualificationTask as TaskClass

    # Reuse the round's real config loading, CLI overrides and all.
    args.corner_speed = None
    config = load_config(TaskClass, args)

    # The obstacle round without pillars is not the obstacle round - it is the
    # qualification round with different speeds - so it gets some by default.
    pillars = args.pillars
    if pillars is None:
        pillars = 4 if args.round == "final" else 0

    if args.sweep:
        key, raw = args.sweep
        values = [float(value) if "." in value else int(value)
                  for value in raw.split(",")]
        run_sweep(TaskClass, config, key, values, max(args.trials, 1),
                  args.seed, args.timeout, pillars)
        raise SystemExit(0)

    if args.trials > 1:
        ok = run_trials(TaskClass, config, args.trials, args.seed, args.timeout,
                        pillars=pillars, parking=args.parking)
        raise SystemExit(0 if ok else 1)

    rng = random.Random(args.seed)
    placement = tuple(args.start) if args.start else random_placement(FieldMap(), rng)
    bay = place_bay(placement, FieldMap(), rng) if args.parking else None
    outcome = simulate(TaskClass, config, placement, window=args.window,
                       timeout_s=args.timeout, seed=args.seed, pillars=pillars,
                       bay=bay)
    print()
    print(describe(outcome))
    if args.window:
        show_window(outcome)
    if outcome["pillars"]:
        print(f"  pillars: {_pillar_summary([outcome])}")
        for pillar in outcome["pillars"]:
            verdict = {True: "correct side", False: "WRONG SIDE",
                       None: "never reached"}[pillar.passed_correctly]
            print(f"    {pillar.color.value:<5} at {pillar.progress / 1000:.2f}m  "
                  f"{verdict:<13} clearance {pillar.min_clearance_mm / 10:+5.1f}cm")
    raise SystemExit(0 if (outcome["completed"] and not outcome["crashed"]
                           and not outcome["pillars_wrong"]
                           and not outcome["pillars_hit"]
                           and (outcome["parking"] is None
                                or outcome["parking"]["parked"])) else 1)

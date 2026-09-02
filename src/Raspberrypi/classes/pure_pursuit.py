"""
Pure pursuit: turn "where I am" plus "where I want to be shortly" into a
steering angle.

The controller draws a circle through the rear axle that passes through a
target point a fixed distance ahead on the path, and steers the arc of that
circle. There is no error integral and no gain to fight - the only knob that
really matters is the lookahead distance:

    short lookahead   follows the line tightly, oscillates when fast
    long lookahead    smooth and stable, cuts corners

which is why it is scaled with speed rather than fixed: fast on the straights
wants a long lookahead, slow in a corner wants a short one.

Two conversions are easy to get wrong and are handled explicitly here:

  * pure pursuit is defined from the REAR AXLE, not the middle of the robot.
    NavigationManager already reports the axle (the lidar is described as an
    offset ahead of it - see classes/robot_geometry.py), so
    `rear_axle_offset_mm` stays at 0. It is only here for a pose source
    whose point is somewhere else on the body.
  * MotorManager.steer() takes a servo command, not a road-wheel angle. The
    two are proportional but not equal, so `max_road_wheel_deg` (measure it:
    turn the wheel to full lock and put a protractor on it) maps one to the
    other.
"""
import math

from utils.angle_utils import angle_difference, clamp

# ============================================================================
# Defaults - every one of these is overridable from a round's config.toml
# ============================================================================
WHEELBASE_MM = 50.0             # front axle to rear axle
LOOKAHEAD_MIN_MM = 250.0
LOOKAHEAD_MAX_MM = 700.0
LOOKAHEAD_BASE_MM = 260.0
LOOKAHEAD_PER_SPEED_MM = 3.0     # added per unit of the 0-100 speed command
MAX_ROAD_WHEEL_DEG = 70.0        # actual steer angle at full lock
MAX_STEER_COMMAND = 70.0         # what MotorManager.steer() calls full lock
REAR_AXLE_OFFSET_MM = 0.0        # + forward of the pose origin, usually <= 0
SERVO_LAG_S = 0.0                # first-order time constant of the wheels; 0 = instant

# In a bend, the lookahead is additionally capped at this fraction of the
# bend's own radius. Aiming further round a corner than its radius puts the
# target across the arc rather than along it, so the robot cuts the corner and
# then runs wide correcting - the classic pure-pursuit weave. Below ~0.5 the
# robot starts chasing its own nose and oscillates the other way.
CORNER_LOOKAHEAD_FRACTION = 0.8
ABSOLUTE_MIN_LOOKAHEAD_MM = 120.0


class PurePursuit:
    """
    Stateless steering controller. Give it a pose and a target point in the
    field frame and it returns a steering command for MotorManager.
    """

    def __init__(self, wheelbase_mm=WHEELBASE_MM,
                 lookahead_min_mm=LOOKAHEAD_MIN_MM,
                 lookahead_max_mm=LOOKAHEAD_MAX_MM,
                 lookahead_base_mm=LOOKAHEAD_BASE_MM,
                 lookahead_per_speed_mm=LOOKAHEAD_PER_SPEED_MM,
                 max_road_wheel_deg=MAX_ROAD_WHEEL_DEG,
                 max_steer_command=MAX_STEER_COMMAND,
                 rear_axle_offset_mm=REAR_AXLE_OFFSET_MM,
                 corner_lookahead_fraction=CORNER_LOOKAHEAD_FRACTION,
                 servo_lag_s=SERVO_LAG_S):
        self.wheelbase_mm = float(wheelbase_mm)
        self.lookahead_min_mm = float(lookahead_min_mm)
        self.lookahead_max_mm = float(lookahead_max_mm)
        self.lookahead_base_mm = float(lookahead_base_mm)
        self.lookahead_per_speed_mm = float(lookahead_per_speed_mm)
        self.max_road_wheel_deg = float(max_road_wheel_deg)
        self.max_steer_command = float(max_steer_command)
        self.rear_axle_offset_mm = float(rear_axle_offset_mm)
        self.corner_lookahead_fraction = float(corner_lookahead_fraction)
        self.servo_lag_s = float(servo_lag_s)

        # Kept for the status line and the debug overlay.
        self.last_target = None
        self.last_alpha_deg = 0.0
        self.last_road_wheel_deg = 0.0
        self.last_command = 0.0
        # Where the wheels actually ARE, as opposed to where they were last
        # told to go. See advance_servo.
        self.actual_road_wheel_deg = 0.0

    # ========================================================================
    # GEOMETRY
    # ========================================================================

    def lookahead_distance(self, speed_command, curvature=0.0):
        """
        How far ahead to aim, in mm.

        Linear in speed and clamped, so a stopped robot still has a target to
        turn toward and a flat-out one is not chasing a point under its own
        nose. Then, in a bend, capped against the bend's radius.

        That cap is what stops the weave. Pure pursuit steers the chord to the
        target point; on a corner of radius R, a lookahead longer than R puts
        that chord across the arc instead of along it, so the robot turns in
        early, cuts the apex, and then has to correct hard the other way on
        exit. Scaling lookahead with speed alone does not save you - at speed
        60 the lookahead here is 440mm on a 350mm corner.

        I/O:
            speed_command: the 0-100 speed we are about to drive at
            curvature: 1/radius of the sharpest bend within reach, from
                       RacingLine.max_curvature_between(); 0 on a straight
        """
        distance = self.lookahead_base_mm + self.lookahead_per_speed_mm * abs(speed_command)
        distance = clamp(distance, self.lookahead_min_mm, self.lookahead_max_mm)
        if curvature > 0.0:
            distance = min(distance, self.corner_lookahead_fraction / curvature)
        return max(ABSOLUTE_MIN_LOOKAHEAD_MM, distance)

    # ========================================================================
    # THE SERVO
    # ========================================================================

    def advance_servo(self, dt):
        """
        Moves the modelled wheel angle `dt` further toward the last command,
        and returns the AVERAGE angle over that dt.

        Steering is not a value, it is an actuator: ask for 20 degrees and the
        wheels arrive there about `servo_lag_s` later. Everything that
        dead-reckons off the steering was reading the COMMAND, i.e. assuming
        the wheels teleport - so during every correction, which is exactly when
        it matters, the odometry was told about yaw the robot had not done. The
        filter then believes a heading the robot is not on, pure pursuit
        corrects an error that is not there, the wheels swing back, and the
        robot weaves down the line without ever settling on it. No lookahead
        fixes that, because the fault is in the pose, not in the geometry.

        A first-order lag is the honest model of it: the response measured with
        `test_steering.py --lag` was ~0.35s to 63%, which IS the exponential's
        time constant. Left at 0 this returns the command, which is the old
        behaviour exactly.

        I/O:
            dt: seconds since the last call
            return: mean road-wheel angle over those seconds, in degrees
        """
        target = self.last_road_wheel_deg
        if self.servo_lag_s <= 0.0 or dt <= 0.0:
            self.actual_road_wheel_deg = target
            return target

        start = self.actual_road_wheel_deg
        decay = math.exp(-dt / self.servo_lag_s)
        self.actual_road_wheel_deg = target + (start - target) * decay
        # Integral of target + (start-target)e^(-t/tau) over [0, dt], / dt.
        return target + (start - target) * self.servo_lag_s / dt * (1.0 - decay)

    def mean_road_wheel_deg(self, seconds):
        """
        The average wheel angle the servo will hold over the NEXT `seconds`,
        assuming the last command stands.

        Same curve as advance_servo, run forward instead of back. This is what
        the lag compensation should predict the robot's yaw with: over a lead
        of 0.35s the wheels spend most of it still on their way to the new
        angle, so predicting with the commanded angle over-turns the projected
        pose - which is why lag_compensation_s had to be held at 0.15s, well
        under the real lag, to keep from over-correcting. With the approach
        modelled the full measured lag can be compensated honestly.
        """
        target = self.last_road_wheel_deg
        if self.servo_lag_s <= 0.0 or seconds <= 0.0:
            return target
        start = self.actual_road_wheel_deg
        decay = math.exp(-seconds / self.servo_lag_s)
        return target + (start - target) * self.servo_lag_s / seconds * (1.0 - decay)

    def rear_axle(self, pose):
        """The pose's reference point shifted back onto the rear axle."""
        if not self.rear_axle_offset_mm:
            return pose.x, pose.y
        radians = math.radians(pose.heading)
        return (pose.x + self.rear_axle_offset_mm * math.sin(radians),
                pose.y + self.rear_axle_offset_mm * math.cos(radians))

    def steering(self, pose, target_xy):
        """
        Steering command that arcs the robot onto `target_xy`.

        I/O:
            pose: Pose from NavigationManager (field mm, heading CW from +Y)
            target_xy: the point to chase, in field mm
            return: steering command in MotorManager's units, - left / + right

        The curvature that passes through the target is 2*sin(alpha)/L_d, where
        alpha is how far off the nose the target sits. Feeding that through the
        bicycle model gives the road-wheel angle; the rest is unit conversion.
        """
        axle_x, axle_y = self.rear_axle(pose)
        offset_x = target_xy[0] - axle_x
        offset_y = target_xy[1] - axle_y
        distance = math.hypot(offset_x, offset_y)

        self.last_target = tuple(target_xy)
        if distance < 1.0:
            # Sitting on the target: no arc is defined, so hold the wheel.
            self.last_alpha_deg = self.last_road_wheel_deg = self.last_command = 0.0
            return 0.0

        bearing = math.degrees(math.atan2(offset_x, offset_y))
        alpha = angle_difference(bearing, pose.heading)

        curvature = 2.0 * math.sin(math.radians(alpha)) / distance
        road_wheel = math.degrees(math.atan(self.wheelbase_mm * curvature))
        road_wheel = clamp(road_wheel, -self.max_road_wheel_deg, self.max_road_wheel_deg)

        command = road_wheel / self.max_road_wheel_deg * self.max_steer_command

        self.last_alpha_deg = alpha
        self.last_road_wheel_deg = road_wheel
        self.last_command = command
        return command

    def set_road_wheel_command(self, command):
        """
        Records a steering command that did NOT come from steering().

        The parking manoeuvre drives the wheels itself, but the odometry still
        has to know what they are doing: PathDrivingTask._turned() dead-reckons
        yaw between lidar scans off the wheel angle, which advance_servo tracks
        from whatever was last commanded. Without this the filter is told the
        robot went straight through every arc of the park, and the pose walks
        away exactly where it is needed most.

        Exactly the inverse of the conversion at the end of steering(), so
        both paths dead-reckon off the same calibration.

        I/O:
            command: steering command in MotorManager units
            return: the road-wheel angle it corresponds to, in degrees
        """
        command = clamp(command, -self.max_steer_command, self.max_steer_command)
        self.last_command = command
        self.last_road_wheel_deg = (command / self.max_steer_command
                                    * self.max_road_wheel_deg)
        return self.last_road_wheel_deg

    # ========================================================================
    # LIMITS
    # ========================================================================

    @property
    def min_turn_radius_mm(self):
        """
        Tightest circle the robot can drive. A corner radius below this cannot
        be followed however good the controller is, so RacingLine's corner
        radius must stay above it - PathDrivingTask checks this at setup.
        """
        return self.wheelbase_mm / math.tan(math.radians(self.max_road_wheel_deg))

    def status_line(self):
        return (f"steer={self.last_command:+5.1f} (wheel {self.actual_road_wheel_deg:+5.1f}"
                f"->{self.last_road_wheel_deg:+5.1f}deg) "
                f"alpha={self.last_alpha_deg:+6.1f}deg")

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
    `rear_axle_offset_mm` shifts the pose back to it. Left at 0 the robot
    understeers slightly on tight corners.
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
WHEELBASE_MM = 165.0             # front axle to rear axle
LOOKAHEAD_MIN_MM = 250.0
LOOKAHEAD_MAX_MM = 700.0
LOOKAHEAD_BASE_MM = 260.0
LOOKAHEAD_PER_SPEED_MM = 3.0     # added per unit of the 0-100 speed command
MAX_ROAD_WHEEL_DEG = 30.0        # actual steer angle at full lock
MAX_STEER_COMMAND = 80.0         # what MotorManager.steer() calls full lock
REAR_AXLE_OFFSET_MM = 0.0        # + forward of the pose origin, usually <= 0

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
                 corner_lookahead_fraction=CORNER_LOOKAHEAD_FRACTION):
        self.wheelbase_mm = float(wheelbase_mm)
        self.lookahead_min_mm = float(lookahead_min_mm)
        self.lookahead_max_mm = float(lookahead_max_mm)
        self.lookahead_base_mm = float(lookahead_base_mm)
        self.lookahead_per_speed_mm = float(lookahead_per_speed_mm)
        self.max_road_wheel_deg = float(max_road_wheel_deg)
        self.max_steer_command = float(max_steer_command)
        self.rear_axle_offset_mm = float(rear_axle_offset_mm)
        self.corner_lookahead_fraction = float(corner_lookahead_fraction)

        # Kept for the status line and the debug overlay.
        self.last_target = None
        self.last_alpha_deg = 0.0
        self.last_road_wheel_deg = 0.0
        self.last_command = 0.0

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
        return (f"steer={self.last_command:+5.1f} (wheel {self.last_road_wheel_deg:+5.1f}deg) "
                f"alpha={self.last_alpha_deg:+6.1f}deg")

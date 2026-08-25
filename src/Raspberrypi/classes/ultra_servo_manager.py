from utils.angle_utils import clamp

# ============================================================================
# Hardware wiring
# ============================================================================
SERVO_CHANNEL = 0          # expansion board PWM channel the servo sits on
ULTRA_ADC_CHANNEL = 0      # expansion board ADC channel the URM09 sits on

SERVO_CENTER_DEG = 45      # servo angle that points the sensor straight ahead
SERVO_LEFT_DEG = 90        # ... at the left-hand wall
SERVO_RIGHT_DEG = 0        # ... at the right-hand wall
SERVO_MIN_DEG = 0
SERVO_MAX_DEG = 50

# How hard the head counter-rotates against the robot's heading error, so it
# keeps pointing at the same patch of wall while the body yaws.
TRACKING_GAIN = 2.0
TRACKING_DEADBAND_DEG = 3

# URM09 analog: full scale is 500cm across the supply voltage.
URM09_MAX_RANGE_CM = 500.0
VIN_MV = 5000              # 3300 if the URM09 is powered from the 3.3V rail


class UltraServoManager:
    """
    The steerable ultrasonic head: a servo on the expansion board carrying a
    URM09 whose analog output comes back on an ADC channel of the same board.

    The head points at the wall the robot is following. As the body yaws off
    its target heading the head rotates the opposite way (update_tracking), so
    the beam stays on the wall instead of sweeping along it - a wall reading
    taken at an angle reads longer than the true perpendicular distance and
    would otherwise fight the wall-following controller.

    read_raw() is in ADC millivolts, which is the unit the wall-following
    constants are tuned in. read_distance_cm() converts to centimeters.
    """

    def __init__(self, board_manager, debug=False,
                 servo_channel=SERVO_CHANNEL, adc_channel=ULTRA_ADC_CHANNEL):
        self.board_manager = board_manager
        self.debug = debug
        self.servo_channel = servo_channel
        self.adc_channel = adc_channel

        self.servo = None
        self.angle = SERVO_CENTER_DEG
        self.base_angle = SERVO_CENTER_DEG   # where the head sits at zero yaw error

    # ========================================================================
    # LIFECYCLE
    # ========================================================================

    def start(self):
        self.servo = self.board_manager.create_servo()
        self.center()
        if self.debug:
            print(f"[ultra] servo on channel {self.servo_channel}, "
                  f"URM09 on ADC {self.adc_channel}")

    def stop(self):
        try:
            self.center()
        except Exception:
            pass
        self.servo = None

    # ========================================================================
    # POINTING
    # ========================================================================

    def move(self, angle):
        if self.servo is None:
            raise RuntimeError("UltraServoManager.start() must be called first")
        self.angle = clamp(angle, SERVO_MIN_DEG, SERVO_MAX_DEG)
        self.servo.move(self.servo_channel, int(self.angle))

    def center(self):
        self.base_angle = SERVO_CENTER_DEG
        self.move(SERVO_CENTER_DEG)

    def point_at_wall(self, turn_direction):
        """
        Aims the head at the wall the robot follows for this lap direction.
        "L" (counter-clockwise laps) follows the left wall, "R" the right.
        """
        self.base_angle = SERVO_LEFT_DEG if turn_direction == "L" else SERVO_RIGHT_DEG
        self.move(self.base_angle)

    def update_tracking(self, heading_offset, deadband_deg=TRACKING_DEADBAND_DEG):
        """
        Counter-rotates the head by the robot's heading error so the beam
        stays on the same patch of wall. Small errors inside the deadband are
        ignored, otherwise the head would buzz constantly on sensor noise.
        """
        if self.servo is None:
            return
        offset = heading_offset if abs(heading_offset) > deadband_deg else 0
        self.move(self.base_angle - offset * TRACKING_GAIN)

    # ========================================================================
    # RANGING
    # ========================================================================

    def read_raw(self):
        """Raw ADC millivolts from the URM09, or None if the read failed."""
        return self.board_manager.read_adc(self.adc_channel)

    def read_distance_cm(self):
        """Distance in centimeters, or None if the read failed."""
        raw = self.read_raw()
        if raw is None:
            return None
        return raw * URM09_MAX_RANGE_CM / VIN_MV

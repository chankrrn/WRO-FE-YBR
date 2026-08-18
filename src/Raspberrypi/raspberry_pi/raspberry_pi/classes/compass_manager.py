import time

from utils.angle_utils import angle_difference, clamp, normalize_angle

# ============================================================================
# Heading control
# ============================================================================
HEADING_KP = 0.5
MAX_HEADING_CORRECTION = 80
BOOT_SETTLE_S = 1.0

# The BNO055 fuses gyro + accel + mag, so a fresh reading can be wild for the
# first second. Nothing here blocks on full calibration - the robot only needs
# heading RELATIVE to where it started, which the gyro gives immediately.


class CompassManager:
    """
    BNO055 IMU used as a heading reference.

    Everything is relative to `initial_heading` (captured at the start line),
    and `target_heading` is what the navigator wants the robot pointed at.
    `offset` is the signed error between them, refreshed by update().

    Absent or broken hardware is not fatal: heading() returns None, offset
    stays 0 and correction() returns 0, so a task degrades to wall-following
    only instead of crashing.
    """

    def __init__(self, debug=False, heading_kp=HEADING_KP):
        self.debug = debug
        self.heading_kp = heading_kp

        self.sensor = None
        self.initial_heading = None
        self.target_heading = None
        self.offset = 0.0

    # ========================================================================
    # LIFECYCLE
    # ========================================================================

    def start(self):
        """Connects over I2C. Never raises - see the class docstring."""
        try:
            import board as board_pins
            import busio
            import adafruit_bno055

            i2c = busio.I2C(board_pins.SCL, board_pins.SDA)
            self.sensor = adafruit_bno055.BNO055_I2C(i2c)
            time.sleep(BOOT_SETTLE_S)
            if self.debug:
                print(f"[compass] BNO055 up, heading {self.heading()}")
        except Exception as e:
            self.sensor = None
            print(f"WARNING: compass unavailable ({e!r}) - continuing without it")

    @property
    def is_available(self):
        return self.sensor is not None

    def stop(self):
        self.sensor = None

    # ========================================================================
    # READING
    # ========================================================================

    def heading(self):
        """Current heading in degrees [0, 360), or None if unavailable."""
        if self.sensor is None:
            return None
        try:
            euler = self.sensor.euler
        except Exception:
            return None
        if not euler or euler[0] is None:
            return None
        return round(normalize_angle(euler[0]), 1)

    def set_initial_heading(self):
        """
        Captures the heading at the start line and aims at it. Returns the
        heading, or None if the compass could not be read.
        """
        heading = self.heading()
        if heading is None:
            print("WARNING: could not set the initial heading!")
            return None
        self.initial_heading = heading
        self.target_heading = heading
        self.offset = 0.0
        print(f"Initial heading set: {self.initial_heading}°")
        return heading

    def set_target_heading(self, heading):
        self.target_heading = normalize_angle(heading)
        return self.target_heading

    def turn_target(self, degrees):
        """Rotates the target heading by `degrees` (+ clockwise)."""
        base = self.target_heading if self.target_heading is not None else self.heading()
        if base is None:
            return None
        return self.set_target_heading(base + degrees)

    def update(self):
        """
        Refreshes `offset` from the live heading. Call once per control tick,
        before correction(). Returns the offset.
        """
        current = self.heading()
        if current is None or self.target_heading is None:
            self.offset = 0.0
        else:
            self.offset = angle_difference(self.target_heading, current)
        return self.offset

    def correction(self):
        """
        Steering contribution in degrees that pulls the robot back onto
        target_heading (- left / + right), clamped so it can never demand
        more than the steering rack has.
        """
        if self.offset == 0:
            return 0.0
        return clamp(-self.offset * self.heading_kp,
                     -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION)

    def is_on_target(self, tolerance_deg=10):
        return abs(self.offset) <= tolerance_deg

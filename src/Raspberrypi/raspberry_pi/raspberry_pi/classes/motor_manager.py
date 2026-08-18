import glob
import os
import time
from collections import deque

import serial

from utils.angle_utils import clamp

# ============================================================================
# Serial link to the Arduino
# ============================================================================
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200
SERIAL_TIMEOUT_S = 0.1

# The Arduino UNO R4 enumerates as /dev/ttyACM*; matching the USB descriptor
# keeps autodetect off the lidar's CP2102N bridge on /dev/ttyUSB0.
USB_ID_HINTS = ("Arduino", "UNO")

# The Arduino resets when the port is opened and prints a banner when ready.
ARDUINO_BOOT_S = 2.0

MAX_STEER_DEG = 80
STEER_SMOOTHING_WINDOW = 5

# drive() skips the serial round-trip when nothing has meaningfully changed.
# The control loop runs far faster than the robot can respond, so most ticks
# ask for the same thing as the last one; at 50Hz, re-sending it is the
# difference between a loop paced by the code and one paced by the UART.
STEER_DEADBAND_DEG = 1.0
SPEED_DEADBAND = 1

DRIVE_DISTANCE_TIMEOUT_S = 30.0
DONE_REPLY = "t"          # what the Arduino sends when a queued move finishes


class MotorManager:
    """
    Drive motor + steering, spoken to over serial as one command per line:

        "<angle>,<speed>,<distance>\\n"

    where angle is steering in degrees (- left / + right), speed is 0-100, and
    distance is degrees of motor-shaft rotation (0 = keep going until told
    otherwise). The Arduino answers every command with a line, and answers a
    non-zero distance move with "t" once the move has actually finished.
    """

    def __init__(self, port=None, baudrate=DEFAULT_BAUD, debug=False):
        self.port = port or self.autodetect_port()
        self.baudrate = baudrate
        self.debug = debug

        self.serial = None
        self.current_speed = 0
        self.current_angle = 0
        self.steering_history = deque(maxlen=STEER_SMOOTHING_WINDOW)

    # ========================================================================
    # LIFECYCLE
    # ========================================================================

    @staticmethod
    def autodetect_port():
        for path in sorted(glob.glob("/dev/serial/by-id/*")):
            if any(hint in os.path.basename(path) for hint in USB_ID_HINTS):
                return os.path.realpath(path)
        return DEFAULT_PORT

    def start(self):
        self.serial = serial.Serial(self.port, self.baudrate, timeout=SERIAL_TIMEOUT_S)
        time.sleep(ARDUINO_BOOT_S)   # the port opening resets the board
        banner = self.serial.readline().decode(errors="ignore").strip()
        if self.debug:
            print(f"[motor] {self.port} @ {self.baudrate}, arduino says: {banner!r}")
        self.current_speed = 0
        self.current_angle = 0
        self.steering_history.clear()

    def stop_and_close(self):
        try:
            self.stop()
            self.steer_center()
            time.sleep(0.1)
        except Exception:
            pass
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None

    @property
    def is_connected(self):
        return self.serial is not None and self.serial.is_open

    # ========================================================================
    # DRIVING
    # ========================================================================

    def _send(self, angle, speed, distance=0, wait_for_reply=True):
        if self.serial is None:
            raise RuntimeError("MotorManager.start() must be called first")
        self.serial.write(f"{int(angle)},{int(speed)},{int(distance)}\n".encode())
        if wait_for_reply:
            return self.serial.readline().decode(errors="ignore").strip()
        return None

    def drive(self, angle, speed):
        """
        Sets steering and speed together, in ONE message.

        The wire protocol has always carried both, but forward() and steer()
        each send a whole line and then block on the reply - so a control loop
        calling both per tick pays two serial round-trips to say one thing.
        This says it once, and says nothing at all when neither value has moved
        past its deadband.

        I/O:
            return: True if a command actually went out
        """
        angle = clamp(angle, -MAX_STEER_DEG, MAX_STEER_DEG)
        if (abs(angle - self.current_angle) < STEER_DEADBAND_DEG
                and abs(speed - self.current_speed) < SPEED_DEADBAND):
            return False
        self.current_angle = angle
        self.current_speed = speed
        self._send(angle, speed, 0)
        return True

    def forward(self, speed):
        self.current_speed = speed
        self._send(self.current_angle, speed, 0)

    def reverse(self, speed):
        self.forward(-abs(speed))

    def stop(self):
        self.current_speed = 0
        self._send(self.current_angle, 0, 0)

    def steer(self, angle):
        angle = clamp(angle, -MAX_STEER_DEG, MAX_STEER_DEG)
        self.current_angle = angle
        self._send(angle, self.current_speed, 0)

    def steer_smooth(self, target_angle):
        """
        Steers toward the rolling average of the last STEER_SMOOTHING_WINDOW
        requests, so a single noisy sensor frame cannot jerk the wheels.
        """
        self.steering_history.append(clamp(target_angle, -MAX_STEER_DEG, MAX_STEER_DEG))
        self.steer(sum(self.steering_history) / len(self.steering_history))

    def steer_center(self):
        self.steering_history.clear()
        self.steer(0)

    def drive_distance(self, speed, distance, angle=None, max_wait=DRIVE_DISTANCE_TIMEOUT_S):
        """
        Blocking: drives at `speed` for `distance` degrees of motor-shaft
        rotation.

        Reads lines until it actually sees the "t" completion signal rather
        than trusting a single readline() bounded by the port's 0.1s timeout,
        so this stays correct for moves longer than that timeout. Returns None
        if "t" never arrives within max_wait seconds.
        """
        angle = self.current_angle if angle is None else angle
        self.current_angle = angle
        self.current_speed = speed
        self._send(angle, speed, distance, wait_for_reply=False)

        deadline = time.monotonic() + max_wait
        while time.monotonic() < deadline:
            line = self.serial.readline().decode(errors="ignore").strip()
            if line == DONE_REPLY:
                return line
            if line and self.debug:
                print(f"[motor] drive_distance ignoring reply {line!r}")

        print("WARNING: drive_distance timed out waiting for the Arduino's 't'")
        return None

    def wait_for_start(self):
        """
        Blocks until the Arduino reports the physical start button was pressed
        (it sends a line containing "1"). Other lines are printed and ignored.
        """
        print("Waiting for the Arduino start signal...")
        while True:
            line = self.serial.readline().decode(errors="ignore").strip()
            if not line:
                continue
            print(f"Arduino says: {line}")
            if "1" in line or "start" in line.lower():
                print("Start signal received.")
                return

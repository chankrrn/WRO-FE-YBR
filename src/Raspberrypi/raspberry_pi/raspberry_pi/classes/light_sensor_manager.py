import time

# ============================================================================
# Hardware wiring / thresholds
# ============================================================================
BLUE_SENSOR_CHANNEL = 2    # expansion board ADC A2, over the blue line
RED_SENSOR_CHANNEL = 3     # expansion board ADC A3, over the orange line

# Readings DROP below these when the sensor passes over its coloured line.
BLUE_THRESHOLD = 2500
RED_THRESHOLD = 2500

# A line takes a moment to cross and both sensors bounce while over it, so a
# fresh crossing is only accepted this long after the previous one. At racing
# speed the corners are seconds apart, so this cannot swallow a real corner.
LINE_COOLDOWN_S = 1.5


class LightSensorManager:
    """
    The two downward-facing light sensors that spot the coloured corner lines.

    detect_line() is level-triggered: it keeps returning a line for as long as
    the sensor is over it. Use consume_line() to count corners - it only fires
    once per crossing.

    Returns "L" for the blue line (counter-clockwise laps) and "R" for the
    orange line (clockwise), matching the turn_direction used everywhere else.
    """

    def __init__(self, board_manager, debug=False,
                 blue_threshold=BLUE_THRESHOLD, red_threshold=RED_THRESHOLD):
        self.board_manager = board_manager
        self.debug = debug
        self.blue_threshold = blue_threshold
        self.red_threshold = red_threshold
        self._last_line_time = 0.0

    def read_blue(self):
        return self.board_manager.read_adc(BLUE_SENSOR_CHANNEL)

    def read_red(self):
        return self.board_manager.read_adc(RED_SENSOR_CHANNEL)

    def detect_line(self):
        """
        I/O:
            return: "L" over the blue line, "R" over the orange line,
                    None over plain mat (or if a sensor read failed)
        """
        blue = self.read_blue()
        red = self.read_red()

        if blue is None or red is None:
            print("WARNING: light sensor read failed!")
            return None

        if blue < self.blue_threshold:
            return "L"
        if red < self.red_threshold:
            return "R"
        return None

    def consume_line(self, expected=None):
        """
        Edge-triggered detect_line(): returns a line at most once per
        LINE_COOLDOWN_S, so one crossing counts as one corner.

        I/O:
            expected: if given, only this line ("L"/"R") is reported - the
                      robot crosses the OTHER colour's line on the far side of
                      every corner and must not count it
            return: "L"/"R" on a fresh crossing, otherwise None
        """
        line = self.detect_line()
        if line is None or (expected is not None and line != expected):
            return None

        now = time.monotonic()
        if now - self._last_line_time < LINE_COOLDOWN_S:
            return None

        self._last_line_time = now
        if self.debug:
            print(f"[light] line crossing: {line}")
        return line

    def reset_cooldown(self):
        self._last_line_time = time.monotonic()

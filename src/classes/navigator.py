from utils.angle_utils import clamp, normalize_angle

# ============================================================================
# Wall following
# ============================================================================
TARGET_WALL_DISTANCE = 300     # raw ADC millivolts from the URM09, not cm
KP_WALL = 0.02
# The original code declared KD_WALL = 0.015 but never applied it. The
# derivative term is wired up now; raise this to damp the oscillation you get
# from a high KP_WALL, but retune KP_WALL with it.
KD_WALL = 0.0

# A reading this far past target means the head is pointed down the mat rather
# than at a wall (a corner opening up, or the beam missing), so the wall term
# is dropped instead of yanking the wheels toward a wall that is not there.
WALL_ERROR_IGNORE = 4000

# When the robot is this far off its heading, getting the nose back on target
# matters more than holding wall distance, so the wall term stands down.
HEADING_PRIORITY_DEG = 20

HEADING_WEIGHT = 0.8
MAX_STEER_DEG = 80

# ============================================================================
# Course layout
# ============================================================================
SECTIONS_PER_LAP = 4           # one corner line per section
TURN_DEGREES = 90


class Navigator:
    """
    Turns the compass and the ultrasonic head into a steering angle, and keeps
    the lap/corner bookkeeping.

    Steering blends two terms:
      - wall following: hold TARGET_WALL_DISTANCE from the followed wall
      - heading hold: pull back onto the target heading, which steps by 90°
        at each corner

    Heading targets are always computed from `initial_heading` plus 90° per
    corner rather than by adding 90° to the previous target, so a corner
    counted slightly early or late cannot accumulate into a drifting course.
    """

    def __init__(self, compass, ultra, laps_goal=3, debug=False):
        self.compass = compass
        self.ultra = ultra
        self.laps_goal = laps_goal
        self.debug = debug

        self.turn_direction = None     # "L" or "R", learned from the first line
        self.turn_count = 0
        self._previous_wall_error = 0.0

    # ========================================================================
    # COURSE STATE
    # ========================================================================

    @property
    def turns_needed(self):
        return self.laps_goal * SECTIONS_PER_LAP

    @property
    def lap_count(self):
        return self.turn_count // SECTIONS_PER_LAP

    @property
    def is_finished(self):
        return self.turn_count >= self.turns_needed

    def set_turn_direction(self, direction):
        """
        Locks in which way the robot is going round, from the first coloured
        line it crosses, and points the ultrasonic head at that wall.
        """
        self.turn_direction = direction
        self.ultra.point_at_wall(direction)
        print(f"Turn direction: {direction} "
              f"({'counter-clockwise' if direction == 'L' else 'clockwise'})")

    def register_turn(self):
        """Counts a corner and steps the target heading 90° round."""
        self.turn_count += 1
        sign = 1 if self.turn_direction == "L" else -1
        if self.compass.initial_heading is not None:
            target = normalize_angle(
                self.compass.initial_heading + sign * TURN_DEGREES * self.turn_count)
            self.compass.set_target_heading(target)
        lap = (self.turn_count - 1) // SECTIONS_PER_LAP + 1   # the lap this corner belongs to
        print(f"Corner {self.turn_count}/{self.turns_needed} (lap {lap}) "
              f"-> target heading {self.compass.target_heading}")

    # ========================================================================
    # STEERING
    # ========================================================================

    def calculate_steering(self):
        """
        I/O:
            return: steering angle in degrees, - left / + right, clamped to
                    the rack's +/-MAX_STEER_DEG
        Call compass.update() first - this reads the offset it computed.
        """
        heading_correction = self.compass.correction()
        steering = self._wall_term(heading_correction)
        steering += heading_correction * HEADING_WEIGHT
        return clamp(steering, -MAX_STEER_DEG, MAX_STEER_DEG)

    def _wall_term(self, heading_correction):
        if self.turn_direction is None:
            return 0.0

        wall_distance = self.ultra.read_raw()
        if wall_distance is None:
            return 0.0

        error = wall_distance - TARGET_WALL_DISTANCE
        derivative = error - self._previous_wall_error
        self._previous_wall_error = error

        if error > WALL_ERROR_IGNORE or abs(heading_correction) > HEADING_PRIORITY_DEG:
            return 0.0

        term = error * KP_WALL + derivative * KD_WALL
        # Following the left wall, drifting far from it must steer right, and
        # the other way round on the right wall.
        return term if self.turn_direction == "L" else -term

    def status_line(self):
        return (f"heading={self.compass.heading()} target={self.compass.target_heading} "
                f"offset={round(self.compass.offset)} wall={self.ultra.read_raw()} "
                f"corner={self.turn_count}/{self.turns_needed}")

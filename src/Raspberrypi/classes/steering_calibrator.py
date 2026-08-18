"""
Works out what the steering really does, from ordinary driving.

`pursuit.max_road_wheel_deg` scales every steering command, so getting it wrong
makes the robot consistently over- or under-steer no matter how well everything
else is tuned. Measuring it with a set-piece manoeuvre needs clear space and,
on a drivetrain that needs a good shove to break stiction, a speed the robot
cannot hold in a corridor.

So do not use a manoeuvre. During any lap the robot is already producing
exactly the data required: a steering command, a yaw rate and a speed. The
bicycle model ties them together,

    yaw_rate = v / L * tan(delta)   ->   delta = atan(yaw_rate * L / v)

and the controller's own command says what fraction of full lock it asked for.
Fitting one against the other over a lap gives the full-lock angle, free, at
the speeds the robot actually drives at.

Doing this instant-by-instant does not work, and the reason is worth stating.
The wheels lag the command by a tenth of a second or so, so pairing a command
with the yaw rate measured at the same moment compares a cause against an
effect that has not arrived. That is regression dilution, and it drags the fit
toward zero: on the simulator it reported 30 degrees for a robot whose real
full lock was 45. Gating on "wait until the command holds still" fixes the bias
but starves the fit, because a badly calibrated robot is exactly the twitchy
one that never holds still - the worse the error, the less data to measure it.

So integrate instead. Over a whole run,

    total heading change = SUM over samples of (distance / L) * tan(delta)

and lag cannot touch that, because a delay only moves a bit of turning from one
sample to the next - the total is conserved. Better still, both sides telescope:
the total heading change is just (final heading - start heading) plus the laps
between, and the summed distance is the path length, so per-sample pose noise
cancels almost entirely. One scalar solve for the full-lock angle then uses
every moving sample, steady or not.

Two things still matter:

  * Use the RAW filter pose, never the extrapolated one. The extrapolated pose
    is dead-reckoned using the current max_road_wheel_deg, so fitting to it
    would just return the value already assumed - a perfect, meaningless fit.
  * Both distance and heading are measured, so a wrong mm_per_s_at_full cannot
    bias the answer.
"""
import math

from utils.angle_utils import angle_difference

# A sample only carries information if the robot is actually moving.
MIN_SPEED_MM_S = 60.0
MAX_DT_S = 0.5                   # a longer gap means we missed filter updates
MIN_SAMPLES = 40

# The run has to contain real cornering, or there is nothing to measure: a
# robot driven in a straight line turns the same amount (none) whatever its
# steering geometry. Rather less than one lap of the loop, which is 4 x 90.
MIN_TOTAL_TURN_DEG = 200.0

# Bracket for the solve, in degrees of full lock. Wide enough for any sane
# steering rack; the answer is bisected inside it.
SEARCH_MIN_DEG = 2.0
SEARCH_MAX_DEG = 70.0


class SteeringCalibrator:
    """
    Accumulates (steering command, road-wheel angle) pairs from live driving
    and fits the full-lock angle through them.

    Usage - one call per tick, then read it at the end of the run:
        calibrator.observe(nav.get_pose(max_age_s=None, extrapolate=False),
                           steer_command)
        ...
        print(calibrator.report())
    """

    def __init__(self, wheelbase_mm=165.0, max_steer_command=80.0,
                 assumed_max_road_wheel_deg=None):
        self.wheelbase_mm = float(wheelbase_mm)
        self.max_steer_command = float(max_steer_command)
        self.assumed = assumed_max_road_wheel_deg

        self._previous = None
        self._samples = 0
        self._skipped_slow = 0

        # (distance / wheelbase, command fraction) per sample. The solve needs
        # them all at once, and a few thousand floats is nothing.
        self._arcs = []
        self._total_turn_rad = 0.0
        self._distance_mm = 0.0

    # ========================================================================
    # COLLECTING
    # ========================================================================

    def observe(self, pose, steer_command):
        """
        Folds one control tick into the fit.

        I/O:
            pose: the RAW filter pose (extrapolate=False) - see the module
                  docstring for why this matters
            steer_command: what was commanded over the interval ending now
        """
        if pose is None:
            return
        previous, self._previous = self._previous, pose
        if previous is None or pose.timestamp <= previous.timestamp:
            return          # no new filter estimate since last time

        dt = pose.timestamp - previous.timestamp
        if dt <= 0.0 or dt > MAX_DT_S:
            return

        distance = math.hypot(pose.x - previous.x, pose.y - previous.y)
        if distance / dt < MIN_SPEED_MM_S:
            self._skipped_slow += 1
            return

        # Both sides of the identity, accumulated. No steadiness test and no
        # rejection by command size: the sum is over the whole run, so a sample
        # taken mid-correction contributes its share of the turning honestly
        # even though its own command and yaw rate do not line up.
        self._total_turn_rad += math.radians(
            angle_difference(pose.heading, previous.heading))
        self._arcs.append((distance / self.wheelbase_mm,
                           steer_command / self.max_steer_command))
        self._distance_mm += distance
        self._samples += 1

    def _predicted_turn(self, full_lock_rad):
        """Total heading change the bicycle model expects for a given full lock."""
        return sum(arc * math.tan(full_lock_rad * fraction)
                   for arc, fraction in self._arcs)

    # ========================================================================
    # READING
    # ========================================================================

    @property
    def samples(self):
        return self._samples

    @property
    def total_turn_deg(self):
        return math.degrees(self._total_turn_rad)

    def estimate(self):
        """
        Full-lock road-wheel angle in degrees, or None if the run did not
        contain enough cornering to tell.

        Solves total_turn = SUM (distance / L) * tan(full_lock * fraction) for
        full_lock. The left side is monotonic in full_lock over the search
        bracket, so plain bisection finds it - no gradients, no initial guess,
        and no way to diverge.
        """
        if self._samples < MIN_SAMPLES:
            return None
        if abs(self.total_turn_deg) < MIN_TOTAL_TURN_DEG:
            return None

        low, high = math.radians(SEARCH_MIN_DEG), math.radians(SEARCH_MAX_DEG)
        target = self._total_turn_rad
        # The identity is signed; solve on magnitudes so a clockwise lap works
        # exactly like a counter-clockwise one.
        sign = 1.0 if target >= 0 else -1.0
        if sign * self._predicted_turn(high) < sign * target:
            return None          # even full lock cannot explain this much turning

        for _ in range(60):
            middle = 0.5 * (low + high)
            if sign * self._predicted_turn(middle) < sign * target:
                low = middle
            else:
                high = middle
        return math.degrees(0.5 * (low + high))

    def report(self):
        """A line for the end of a run, saying what to put in config.toml."""
        measured = self.estimate()
        if measured is None:
            return (f"[steering] not enough cornering to calibrate "
                    f"({self._samples} samples over {self._distance_mm / 1000:.1f}m, "
                    f"turning {self.total_turn_deg:.0f}deg; need {MIN_SAMPLES} samples "
                    f"and {MIN_TOTAL_TURN_DEG:.0f}deg). Drive a full lap.")

        line = (f"[steering] measured pursuit.max_road_wheel_deg = {measured:.1f}"
                f"   ({self._samples} samples, {self._distance_mm / 1000:.1f}m, "
                f"{self.total_turn_deg:.0f}deg turned)")
        # Known bias: the wheels never quite reach the commanded angle during
        # a fast correction, so the commanded integral slightly over-predicts
        # the turning and the solve comes back low. Measured against the
        # simulator across a 2.8x range of true values, it reads 4-9% under.
        line += "\n[steering] reads a few percent low - round up, do not round down"

        if self.assumed:
            ratio = measured / self.assumed
            line += f"\n[steering] config says {self.assumed:.1f}"
            if abs(ratio - 1.0) < 0.15:
                line += " - close enough, leave it"
            else:
                turning = "MORE" if ratio > 1.0 else "LESS"
                line += (f" - so the robot turns {turning} than commanded, by "
                         f"{abs(ratio - 1.0) * 100:.0f}%. Set it to {measured:.1f}.")
        return line

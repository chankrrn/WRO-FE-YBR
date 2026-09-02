"""
A small PID, sized for the two loops the final round cascades together.

The final-round steering law is two of these in series: a wall PID turns a
lateral error against a fitted wall into a heading OFFSET, and a heading PID
turns the resulting heading error into a steer angle. Neither needs anything
elaborate - both start as pure P - but three details are load-bearing:

    OUTPUT CLAMPING, because the heading PID's output is a steer angle and
        the servo has a mechanical stop. Clamping here rather than at the
        motor keeps the integral honest about what was actually commanded.

    ANTI-WINDUP, because the wall term is asked for the impossible whenever
        a pillar squeezes the lane. Without it the integral banks error
        during the squeeze and spends it steering the wrong way afterwards.

    RESET ON RE-ENABLE, which is the whole reason set_active() exists. The
        wall term is switched OFF through a corner - mid-turn there is no
        stable "outer wall" to measure against, so a live lateral loop would
        fight the turn. When it comes back the robot is somewhere else
        entirely, and the pre-turn history is not just stale but misleading.

Ported from the winning team's PIDController, with one addition noted at
`derivative_on_measurement`.
"""
from utils.angle_utils import clamp

# ============================================================================
# Defaults
# ============================================================================
OUTPUT_MIN = -100.0
OUTPUT_MAX = 100.0

# Both loops ship as pure P. That is not laziness - a wall-relative lateral
# error is already an integrating quantity (steering changes the RATE at
# which the error closes, not the error), so the loop has an integrator built
# into the plant. Adding an I term to that is how you get a weave.


class PID:
    """
    P, I and D with clamped output and a clamped integral.

    Inactive by default: construct it, then set_active(True) when the state
    machine wants it. update() returns 0.0 while inactive, so a disabled
    term contributes nothing rather than holding its last value.
    """

    def __init__(self, kp, ki=0.0, kd=0.0,
                 output_min=OUTPUT_MIN, output_max=OUTPUT_MAX,
                 derivative_on_measurement=False):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_min = float(output_min)
        self.output_max = float(output_max)

        # Differentiating the MEASUREMENT instead of the error kills the
        # "derivative kick": our lane table steps the setpoint 500 -> 250mm
        # the instant a pillar is classified, and d(error)/dt would see that
        # step as an infinite rate and slam the steering for one tick. Their
        # code differentiates the error, which is safe only because their D
        # is zero. Ours is too, for now - but the flag is here so that
        # tuning D later does not quietly reintroduce the kick.
        self.derivative_on_measurement = bool(derivative_on_measurement)

        self.integral = 0.0
        self.last_error = 0.0
        self.last_measurement = 0.0
        self.active = False

    # ========================================================================
    # CONTROL
    # ========================================================================
    def update(self, error, dt, measurement=None):
        """
        One step of the loop.

        I/O:
            error: setpoint - measurement, in whatever units the gains expect
            dt: seconds since the last call; <= 0 is treated as no time
                passing and produces no output rather than a divide by zero
            measurement: only needed when derivative_on_measurement is set
            return: the clamped control output, or 0.0 while inactive
        """
        if not self.active or dt <= 0.0:
            return 0.0

        error = float(error)
        self.integral += error * dt

        # Clamp the integral to the span it could legally command on its own.
        # This is the cheap form of anti-windup: it cannot stop the I term
        # saturating, but it stops it accumulating a debt that takes seconds
        # of opposite-sign error to pay back.
        if self.ki > 0.0:
            self.integral = clamp(self.integral,
                                  self.output_min / self.ki,
                                  self.output_max / self.ki)

        if self.derivative_on_measurement and measurement is not None:
            # d(error)/dt == -d(measurement)/dt for a constant setpoint, so
            # the sign flips back here and the gain keeps its usual meaning.
            derivative = -(float(measurement) - self.last_measurement) / dt
            self.last_measurement = float(measurement)
        else:
            derivative = (error - self.last_error) / dt

        self.last_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return clamp(output, self.output_min, self.output_max)

    # ========================================================================
    # STATE
    # ========================================================================
    def reset(self):
        """Forget the integral and the derivative history."""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_measurement = 0.0

    def set_active(self, enable, measurement=None):
        """
        Enable or disable the loop, clearing stale history on the way in.

        Pass `measurement` when re-enabling a loop that differentiates the
        measurement, otherwise the first tick back sees a step from 0 and
        produces exactly the kick this option exists to avoid.
        """
        enable = bool(enable)
        if enable and not self.active:
            self.reset()
            if measurement is not None:
                self.last_measurement = float(measurement)
        self.active = enable

    def set_gains(self, kp, ki, kd):
        """
        Retune, and drop the history that was accumulated under the old gains.

        The adaptive-gain trick stiffens the wall term on a segment with no
        pillars; without this reset the stored integral would be reinterpreted
        at the new Ki and produce a step in the output.
        """
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.reset()

    def __repr__(self):
        state = "on" if self.active else "off"
        return (f"PID(kp={self.kp:g} ki={self.ki:g} kd={self.kd:g} "
                f"{state} i={self.integral:.3f})")

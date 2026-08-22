import time

# Target control PERIOD, not a delay to add on top of the work. The loop
# sleeps until the next deadline, so a tick that took 5ms and a tick that took
# 15ms are followed by different sleeps and arrive at the same cadence.
#
# It matters more than pacing usually does, because dt is not just a schedule
# here - it is an input. PathDrivingTask._travelled(dt) turns it into the
# distance fed to the particle filter's motion model, and _limit_steer_rate
# turns it into how far the servo may move. Sleeping a fixed 20ms AFTER the
# work made the period 20ms + however long the tick ran, so an expensive tick
# both arrived late and told the filter it had driven further than it had.
LOOP_PERIOD_S = 0.02

# How far behind the schedule may fall before the loop resyncs instead of
# trying to catch up. Catching up means running ticks back to back with no
# sleep, which for a control loop is worse than the lateness: the robot has
# not moved any further, so the extra ticks re-steer on the same pose.
MAX_LAG_S = LOOP_PERIOD_S

# WRO gives 3 minutes per attempt; stopping ourselves is tidier than being
# stopped mid-corner with the motor still driving.
DEFAULT_MAX_RUNTIME_S = 180.0


class Task:
    """
    Base for a runnable round.

    Fixed shape of a run:
        startup (done by TaskContext) -> wait for the start button ->
        setup() -> step() until is_finished() -> finish()

    Subclasses implement setup/step/is_finished. Whatever happens - finishing
    normally, Ctrl-C, or an exception - finish() runs and the caller's
    TaskContext shuts the hardware down, so the robot never keeps driving.
    """

    name = "task"
    requires_lidar = False
    requires_camera = False

    def __init__(self, context, max_runtime_s=DEFAULT_MAX_RUNTIME_S,
                 status_every=25):
        self.context = context
        self.max_runtime_s = max_runtime_s
        self.status_every = status_every
        self.tick = 0
        self.start_time = None
        # Ticks that ran longer than LOOP_PERIOD_S. Reported at the end -
        # a loop that overran most of its ticks is not running at the rate
        # everything downstream was tuned for, and that should not be
        # something you have to guess at from the outside.
        self.overruns = 0
        self.worst_tick_ms = 0.0

    # ========================================================================
    # OVERRIDE THESE
    # ========================================================================

    def setup(self):
        """Runs once after the start signal, before the first step()."""

    def step(self):
        """One control tick. Must not block for long."""
        raise NotImplementedError

    def is_finished(self):
        """True when the round is over."""
        return False

    def finish(self):
        """Runs once on the way out, however the run ended."""
        self.context.motor.stop()
        self.context.motor.steer_center()

    def status(self):
        """One-line progress report, printed every `status_every` ticks."""
        return f"tick={self.tick} elapsed={self.elapsed:.1f}s"

    # ========================================================================
    # RUNNER
    # ========================================================================

    @property
    def elapsed(self):
        return 0.0 if self.start_time is None else time.monotonic() - self.start_time

    @property
    def timed_out(self):
        return self.max_runtime_s is not None and self.elapsed > self.max_runtime_s

    def run(self):
        """
        Drives the whole round. Returns True if it finished on its own terms,
        False if it was interrupted or timed out.
        """
        print(f"\n{'=' * 70}\n WRO FUTURE ENGINEERS - {self.name.upper()}\n{'=' * 70}")

        self.context.wait_for_start()
        self.context.compass.set_initial_heading()
        self.start_time = time.monotonic()

        completed = False
        try:
            self.setup()
            next_tick_at = time.monotonic()
            while not self.is_finished():
                if self.timed_out:
                    print(f"Time limit ({self.max_runtime_s}s) reached - stopping.")
                    break
                self.tick += 1
                started_at = time.monotonic()
                self.step()
                if self.status_every and self.tick % self.status_every == 0:
                    print(self.status())

                self.worst_tick_ms = max(self.worst_tick_ms,
                                         (time.monotonic() - started_at) * 1000.0)
                next_tick_at += LOOP_PERIOD_S
                now = time.monotonic()
                if now > next_tick_at + MAX_LAG_S:
                    # Far enough behind that catching up would mean a burst of
                    # zero-sleep ticks. Give up the lost time and resync.
                    self.overruns += 1
                    next_tick_at = now
                else:
                    time.sleep(max(0.0, next_tick_at - now))
            else:
                completed = True
        except KeyboardInterrupt:
            print("\nInterrupted by user.")
        finally:
            try:
                self.finish()
            except Exception as e:
                print(f"WARNING: finish() failed: {e!r}")

        rate = self.tick / self.elapsed if self.elapsed else 0.0
        print(f"{self.name}: {'completed' if completed else 'stopped early'} "
              f"after {self.elapsed:.1f}s  "
              f"({self.tick} ticks, {rate:.1f}Hz, worst tick "
              f"{self.worst_tick_ms:.1f}ms, {self.overruns} overran)")
        return completed

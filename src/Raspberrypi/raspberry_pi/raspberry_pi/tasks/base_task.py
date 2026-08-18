import time

# Nothing in the control loop blocks, so without a sleep it would spin the ADC
# and the serial link far faster than either can answer.
LOOP_DELAY_S = 0.02

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
            while not self.is_finished():
                if self.timed_out:
                    print(f"Time limit ({self.max_runtime_s}s) reached - stopping.")
                    break
                self.tick += 1
                self.step()
                if self.status_every and self.tick % self.status_every == 0:
                    print(self.status())
                time.sleep(LOOP_DELAY_S)
            else:
                completed = True
        except KeyboardInterrupt:
            print("\nInterrupted by user.")
        finally:
            try:
                self.finish()
            except Exception as e:
                print(f"WARNING: finish() failed: {e!r}")

        print(f"{self.name}: {'completed' if completed else 'stopped early'} "
              f"after {self.elapsed:.1f}s")
        return completed

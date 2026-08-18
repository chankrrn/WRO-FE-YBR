"""
Manual steering check: type a road-wheel command, the servo turns to it and
holds until the next one.

No path is driven here - just the motor link - but with --debug the same
navigation view test_navigation.py uses comes up alongside the prompt (the
racing line drawn on it for reference, though nothing follows it), so a bad
lidar mount or a dead compass shows up on the bench instead of costing a
round. --lidar starts the lidar for that view; without it every pose reports
confidence 0, which is still fine for a pure servo check.
"""
from classes.debug_view import DebugView
from classes.motor_manager import MAX_STEER_DEG
from classes.racing_line import RacingLine
from tasks.base_task import Task
from tasks.path_task import DEFAULTS

QUIT_WORDS = ("q", "quit", "exit")
CENTER_WORDS = ("", "c", "center", "centre")


class ServoTestTask(Task):
    """
    Waits for the start button like every other round, then repeatedly asks
    for a steering angle in degrees and holds the servo there. Blocking on
    input() between ticks is fine - nothing else needs this loop to run fast,
    so the debug view (if on) only refreshes once per command, not live.
    """

    name = "servo_test"

    def __init__(self, context, config=None, status_every=0, **kwargs):
        super().__init__(context, status_every=status_every, **kwargs)
        self.config = config
        self._done = False
        self._path = None
        self._debug_view = None

    def setup(self):
        self.context.motor.steer_center()
        print(f"\nEnter a steering angle in degrees (-{MAX_STEER_DEG} to "
              f"{MAX_STEER_DEG}), 'c' to centre, or 'q' to quit.")
        if self.context.debug:
            self._path = RacingLine(field_map=self.context.nav.map,
                                    wall_margin_mm=DEFAULTS["path.wall_margin_mm"],
                                    corner_radius_mm=DEFAULTS["path.corner_radius_mm"],
                                    resolution_mm=DEFAULTS["path.resolution_mm"])
            self._debug_view = DebugView(self.context.nav,
                                         ascii_mode=self.context.ascii_debug)
            self._show_debug()

    def step(self):
        raw = input("angle> ").strip().lower()
        if raw in QUIT_WORDS:
            self._done = True
            return
        if raw in CENTER_WORDS:
            self.context.motor.steer_center()
            print("  centred")
        else:
            try:
                angle = float(raw)
            except ValueError:
                print(f"  not a number: {raw!r}")
                return
            self.context.motor.steer(angle)
            print(f"  -> {self.context.motor.current_angle:.0f} deg")
        self._show_debug()

    def _show_debug(self):
        """One refresh of the nav view - just the pose/path, nothing driven."""
        if self._debug_view is None:
            return
        self.context.nav.update()
        if not self._debug_view.show(draw=self._path.draw):
            self._done = True

    def is_finished(self):
        return self._done

    def finish(self):
        super().finish()
        if self._debug_view is not None:
            self._debug_view.close()

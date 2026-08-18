"""
Live navigation debug view shared by every task: the same field render
test_navigation.py uses (particles, scan, pose), optionally with a path drawn
on top, plus its two controls - ESC closes the window, --ascii swaps it for a
text readout that works over plain SSH.
"""
import cv2

from classes.navigation_manager import CANVAS_MARGIN_PX, CANVAS_PX


class DebugView:
    """
    One field-render window (or ASCII readout) per task, refreshed by show().

    Usage, once per tick:
        if not debug_view.show(draw=extra_drawing_fn):
            # ESC was pressed - fold this into is_finished()
    """

    def __init__(self, nav, ascii_mode=False, window_name="Navigation"):
        self.nav = nav
        self.ascii_mode = ascii_mode
        self.window_name = window_name
        self._window_open = False

    def to_px(self, x, y):
        """Field mm -> render_debug()'s canvas pixels, for drawing on top of it."""
        scale = (CANVAS_PX - 2 * CANVAS_MARGIN_PX) / self.nav.map.field_size_mm
        center = CANVAS_PX // 2
        return int(round(center + x * scale)), int(round(center - y * scale))

    def show(self, draw=None):
        """
        One refresh. `draw(canvas, to_px)`, if given, adds anything extra (a
        path, a pursuit target) on top of the pose/particle render.

        I/O:
            return: False if the user asked to quit (ESC, window mode only) -
                    ascii mode has no such control and always returns True
        """
        if self.ascii_mode:
            print("\033[2J\033[H" + self.nav.debug_text(), flush=True)
            return True

        canvas = self.nav.render_debug()
        if draw is not None:
            draw(canvas, self.to_px)
        try:
            cv2.imshow(self.window_name, canvas)
        except cv2.error as e:
            print(f"Cannot open a window ({e.err.strip()}) - switching to --ascii.")
            self.ascii_mode = True
            return True
        self._window_open = True
        return cv2.waitKey(1) != 27

    def close(self):
        if self._window_open:
            cv2.destroyWindow(self.window_name)
            self._window_open = False

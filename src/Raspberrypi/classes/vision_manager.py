"""
The pillar camera, moved off the control loop.

Detection is the most expensive thing the obstacle round does and the least
urgent. A frame costs a capture that blocks until the sensor delivers, plus
the solve on top; the control loop wants to run every 20ms. Running them on
the same thread means every camera tick stretches that tick to several times
its budget, and the steering, the odometry and the pose extrapolation all
integrate the resulting dt - so the jitter does not just delay the loop, it
degrades it.

Nothing about the round needs detection to happen inside a control tick. The
steering reads `nav.blocks`, which persists between frames by design (see
classes/block_map.py) - it does not read the current frame. So the camera can
run at its own pace on its own thread and simply keep that map up to date,
exactly the way LidarManager already folds scan points into an array the loop
polls.

    vision = VisionManager(camera, object_solver, nav)
    vision.start()          # captures nothing until resume()
    vision.resume()         # once the filter knows where the robot is
    ...
    nav.blocks.confirmed()  # read from the control loop, any time

The thread never raises into the round. A frame that fails is counted and the
loop backs off and tries the next one, because a camera that drops out should
cost the pillars, not the lap.
"""
import threading
import time

# The camera used to run every other control tick, which on a loop that the
# camera itself was stalling worked out at roughly 12Hz. Free-running it
# instead would be faster - but BlockMap's pruning is counted in FRAMES
# (MAX_MISSES), so tripling the frame rate would third the real time a block
# survives being out of shot. Holding the old cadence keeps that tuning
# meaningful; raise it deliberately, with MAX_MISSES, not by accident.
DEFAULT_MIN_PERIOD_S = 1.0 / 15.0

# How long to sit out after a failed frame, so a camera that has gone away
# does not spin a core re-raising the same exception thousands of times.
ERROR_BACKOFF_S = 0.5

# Frames that fail in a row before the thread says so out loud. One dropped
# frame is normal; a hundred means the camera is gone.
FAILURES_BEFORE_WARNING = 20


class VisionManager:
    """
    Runs capture -> detect -> map on a background thread.

    The only shared state is `nav.blocks`, which takes its own lock, and
    `nav._pose`, which is read and never written from here. That is the whole
    of the concurrency story - deliberately, because a competition robot is
    not the place to debug a lock ordering.

    Starts PAUSED. Detections placed with a pose the filter has not converged
    on are rejected by BlockMap anyway, so capturing before the round has
    localized would burn a core to increment a rejection counter; the task
    resumes it once it knows where the robot is.
    """

    def __init__(self, camera, object_solver, nav, debug=False,
                 min_period_s=DEFAULT_MIN_PERIOD_S):
        self.camera = camera
        self.solver = object_solver
        self.nav = nav
        self.debug = debug
        self.min_period_s = float(min_period_s)

        self._thread = None
        self._stop_event = threading.Event()
        self._resumed = threading.Event()
        self._error = None

        # Off for the whole lap; the park switches it on. See watch_for_parking.
        self.watch_parking = False
        self._parking_lock = threading.Lock()
        self._parking_marks = []
        self._parking_seen_at = 0.0

        self._frames = 0
        self._failures = 0
        self._consecutive_failures = 0
        self._first_frame_at = 0.0
        self._last_frame_at = 0.0
        self._frame_ms = 0.0

    # ========================================================================
    # LIFECYCLE
    # ========================================================================

    def start(self):
        """Starts the thread. It captures nothing until resume() is called."""
        if self.is_running:
            return self
        if self.camera is None or self.solver is None:
            raise RuntimeError("VisionManager needs both a camera and a solver")

        self._error = None
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="vision", daemon=True)
        self._thread.start()
        if self.debug:
            print(f"[vision] thread up, paused, {1.0 / self.min_period_s:.0f}Hz cap")
        return self

    def stop(self):
        """
        Stops the thread and waits for the frame in flight.

        The join timeout is generous on purpose: the thread can be inside a
        blocking capture, and killing the round while the camera still owns a
        buffer is how a shutdown hangs. It is a daemon thread, so even a
        timeout here cannot keep the process alive.
        """
        self._stop_event.set()
        self._resumed.set()          # wake it if it is parked in pause()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self.debug:
            print(f"[vision] stopped after {self._frames} frames, "
                  f"{self._failures} failed")

    def resume(self):
        """Starts actually capturing. Safe to call more than once."""
        if not self._resumed.is_set() and self.debug:
            print("[vision] capturing")
        self._resumed.set()

    def pause(self):
        """Stops capturing without stopping the thread."""
        self._resumed.clear()

    @property
    def is_running(self):
        return self._thread is not None and self._thread.is_alive()

    @property
    def is_capturing(self):
        return self.is_running and self._resumed.is_set()

    @property
    def error(self):
        """The last exception a frame raised, or None. Never fatal."""
        return self._error

    @property
    def frames(self):
        return self._frames

    @property
    def fps(self):
        elapsed = self._last_frame_at - self._first_frame_at
        return 0.0 if elapsed <= 0.0 else (self._frames - 1) / elapsed

    def wait_until_ready(self, timeout=3.0):
        """
        Blocks until the first frame has been through the pipeline. Returns
        False on timeout, which means the camera is not delivering - worth
        saying at startup rather than discovering as an empty block map.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._frames:
                return True
            if not self.is_running:
                return False
            time.sleep(0.02)
        return False

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False

    # ========================================================================
    # THE THREAD
    # ========================================================================

    def _run(self):
        while not self._stop_event.is_set():
            if not self._resumed.is_set():
                self._resumed.wait(timeout=0.1)
                continue

            started_at = time.monotonic()
            try:
                self._capture_and_map()
            except Exception as e:      # noqa: BLE001 - a bad frame is not fatal
                self._note_failure(e)
                self._stop_event.wait(ERROR_BACKOFF_S)
                continue

            # Pace to min_period_s counting the work, not on top of it, so the
            # cadence is the one asked for however long a frame took.
            self._stop_event.wait(max(0.0, self.min_period_s
                                      - (time.monotonic() - started_at)))

    def _capture_and_map(self):
        started_at = time.monotonic()
        # The display copy is only worth taking when the solver will draw on it.
        hsv = self.camera.capture_for_blocks(with_display=self.solver.debug)
        if hsv is None:
            return
        detections = self.solver.detect(hsv, display_image=self.camera.display_image)
        self.nav.observe_blocks(detections)
        if self.watch_parking:
            # Only while parking. A third colour mask on every frame is a third
            # more work in the camera loop, and for the whole lap there is
            # nothing to look for - the bay only matters once the laps are done.
            with self._parking_lock:
                self._parking_marks = self.solver.detect_parking(hsv)
                self._parking_seen_at = time.monotonic()

        now = time.monotonic()
        self._frame_ms = (now - started_at) * 1000.0
        self._frames += 1
        self._last_frame_at = now
        if self._frames == 1:
            self._first_frame_at = now
        self._consecutive_failures = 0

    def _note_failure(self, error):
        self._error = error
        self._failures += 1
        self._consecutive_failures += 1
        if (self._consecutive_failures == 1
                or self._consecutive_failures % FAILURES_BEFORE_WARNING == 0):
            print(f"WARNING: vision frame failed ({self._consecutive_failures} in a "
                  f"row): {error!r}")

    # ========================================================================
    # REPORTING
    # ========================================================================

    def watch_for_parking(self, enabled=True):
        """Start (or stop) looking for the pink bay walls in each frame."""
        self.watch_parking = bool(enabled)
        if not enabled:
            with self._parking_lock:
                self._parking_marks = []

    def parking_marks(self, max_age_s=0.6):
        """
        The pink bay walls the camera can see right now, biggest first.

        Empty when the camera has not looked recently enough to be believed -
        a stale sighting is worse than none, because the thing it is used for
        is deciding that the bay is HERE.
        """
        with self._parking_lock:
            if not self._parking_marks:
                return []
            if time.monotonic() - self._parking_seen_at > max_age_s:
                return []
            return list(self._parking_marks)

    def _parking_status(self):
        """What the park is being handed, for the status line."""
        if not self.watch_parking:
            return ""
        marks = self.parking_marks()
        if not marks:
            return "  bay:none"
        return "  bay:" + ",".join(f"{m.bearing_deg:+.0f}deg" for m in marks[:2])

    def status_line(self):
        if not self.is_running:
            return "vision off"
        if not self._resumed.is_set():
            return "vision paused"
        return (f"vision {self.fps:.1f}fps {self._frame_ms:.1f}ms"
                + (f" {self._failures} failed" if self._failures else "")
                + self._parking_status())

import asyncio
import glob
import math
import os
import threading
import time

import cv2
import numpy as np

from rplidarc1 import RPLidar

from utils.image_drawing_utils import ImageDrawingUtils

# ============================================================================
# Device configuration
# ============================================================================
DEFAULT_BAUDRATE = 460800        # fixed for the C1
DEFAULT_PORT = "/dev/ttyUSB0"

# The C1's USB adapter is a Silicon Labs CP2102N. Matching on that keeps the
# autodetect from grabbing the Arduino, which enumerates as /dev/ttyACM*.
USB_ID_HINTS = ("CP2102", "Silicon_Labs")

# Datasheet range of the C1. Readings outside this are noise/garbled packets.
MIN_RANGE_MM = 50.0
MAX_RANGE_MM = 12000.0

# A reading older than this is dropped from the scan. The C1 spins at ~10Hz,
# so every angle should refresh ~10x/second - anything a half second old means
# that direction is currently returning nothing (too far, too dark, or the
# beam is hitting glass).
DEFAULT_STALE_AFTER_S = 0.5

# ============================================================================
# Debug visualizer configuration
# ============================================================================
CANVAS_PX = 700
CANVAS_MARGIN_PX = 30
RENDER_RANGE_MM = 4000.0         # edge of the canvas
RING_STEP_MM = 500               # smallest gap between range rings
RING_TARGET_COUNT = 8

# Named sectors printed in the debug overlay: (label, start_deg, end_deg)
SUMMARY_SECTORS = (
    ("FRONT", -20, 20),
    ("RIGHT", 70, 110),
    ("BACK", 160, 200),
    ("LEFT", 250, 290),
)


class LidarManager:
    """
    Wraps the async rplidarc1 driver in a plain synchronous API so the main
    control loop can just poll it.

    The driver is async-only, so the scan runs on its own asyncio loop in a
    background thread. That thread folds every incoming point into a
    360-element array indexed by whole degrees - one slot per degree, holding
    the most recent distance seen for that direction. Reading the "scan" is
    therefore always instant and never blocks on a rotation completing.

    Note this is a rolling snapshot, NOT a single synchronized sweep: with the
    C1 spinning at ~10Hz, the oldest slots in a returned scan are up to ~100ms
    older than the newest ones. That is fine for obstacle distances on a slow
    robot, but do not treat the array as one instantaneous picture of the room
    if the robot is moving fast.

    Angle convention (matches ObjectSolver's bearing):
        0    = straight ahead
        +90  = robot's right
        -90 / 270 = robot's left
        180  = behind
    Accessors accept negative angles. `angle_offset_deg` rotates the sensor's
    own zero (the mark on the C1's housing) onto the robot's forward axis, and
    `mirror` flips the direction of rotation for an upside-down mount.
    """

    def __init__(self, port=None, baudrate=DEFAULT_BAUDRATE, debug=False,
                 angle_offset_deg=0.0, mirror=False,
                 min_distance_mm=MIN_RANGE_MM, max_distance_mm=MAX_RANGE_MM,
                 stale_after_s=DEFAULT_STALE_AFTER_S,
                 render_range_mm=RENDER_RANGE_MM):
        self.port = port or self.autodetect_port()
        self.baudrate = baudrate
        self.debug = debug
        self.angle_offset_deg = angle_offset_deg
        self.mirror = mirror
        self.min_distance_mm = min_distance_mm
        self.max_distance_mm = max_distance_mm
        self.stale_after_s = stale_after_s
        self.render_range_mm = render_range_mm   # distance at the canvas edge

        self._lidar = None
        self._thread = None
        self._stop_event = threading.Event()
        self._error = None            # exception raised inside the scan thread

        self._lock = threading.Lock()
        self._distances = np.full(360, np.nan)   # mm, index = degree
        self._updated_at = np.zeros(360)          # time.monotonic() per slot

        self._points_received = 0
        self._rate_window_start = 0.0
        self._rate_window_points = 0
        self._points_per_second = 0.0

    # ========================================================================
    # LIFECYCLE
    # ========================================================================

    @staticmethod
    def autodetect_port():
        """
        Finds the C1's serial port by USB descriptor, falling back to
        DEFAULT_PORT. Needed because /dev/ttyUSB0 is only stable while the C1
        is the sole USB-serial device plugged in.
        """
        for path in sorted(glob.glob("/dev/serial/by-id/*")):
            if any(hint in os.path.basename(path) for hint in USB_ID_HINTS):
                return os.path.realpath(path)
        return DEFAULT_PORT

    def start(self):
        """
        Opens the serial port, healthchecks the device and starts scanning in
        the background. Raises (ConnectionError etc.) on the calling thread if
        the device does not come up, so a failure is visible immediately.
        """
        if self.is_running:
            return

        self._error = None
        self._stop_event.clear()
        if self.debug:
            print(f"[lidar] connecting to {self.port} @ {self.baudrate}")

        # Constructing RPLidar connects + healthchecks synchronously, so any
        # wiring problem surfaces here rather than inside the thread.
        self._lidar = RPLidar(self.port, self.baudrate)

        self._thread = threading.Thread(target=self._run, name="lidar", daemon=True)
        self._thread.start()
        if self.debug:
            print("[lidar] scanning")

    def stop(self):
        """Stops the scan, sends the motor stop command and closes the port."""
        self._stop_event.set()
        if self._lidar is not None:
            # The driver's coroutine polls this same flag to exit its read loop.
            self._lidar.stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        self._lidar = None
        if self.debug:
            print(f"[lidar] stopped after {self._points_received} points")

    @property
    def is_running(self):
        return self._thread is not None and self._thread.is_alive()

    @property
    def error(self):
        """The exception that killed the scan thread, or None."""
        return self._error

    def wait_until_ready(self, timeout=5.0, min_angles=60):
        """
        Blocks until at least `min_angles` directions have a fresh reading.
        Returns False on timeout, which usually means the motor is not
        spinning or the device is pointed into open space.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._error is not None:
                raise self._error
            if np.count_nonzero(~np.isnan(self.get_scan())) >= min_angles:
                return True
            time.sleep(0.05)
        return False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False

    # ========================================================================
    # SCAN THREAD
    # ========================================================================

    def _run(self):
        lidar = self._lidar   # stop() clears self._lidar once we are done
        try:
            asyncio.run(self._scan_loop())
        except Exception as e:      # noqa: BLE001 - surfaced via self.error
            self._error = e
            if self.debug:
                print(f"[lidar] scan thread died: {e!r}")
        finally:
            try:
                lidar.shutdown()
            except Exception:
                pass

    async def _scan_loop(self):
        # simple_scan() sends the SCAN command and returns the driver's reader
        # coroutine; it feeds lidar.output_queue, which _consume() drains.
        scan = self._lidar.simple_scan()
        consumer = self._consume()
        await asyncio.gather(scan, consumer)

    async def _consume(self):
        queue = self._lidar.output_queue
        while not self._stop_event.is_set():
            try:
                batch = [await asyncio.wait_for(queue.get(), timeout=0.2)]
            except asyncio.TimeoutError:
                continue
            # Drain whatever else has piled up in one go - taking the lock
            # 5000 times a second (the C1's sample rate) would not be free.
            while not queue.empty() and len(batch) < 1024:
                batch.append(queue.get_nowait())
            self._apply(batch)

    def _apply(self, batch):
        now = time.monotonic()
        accepted = 0
        with self._lock:
            for point in batch:
                distance = point["d_mm"]
                # The driver reports an out-of-range/no-return sample as None.
                if distance is None:
                    continue
                if not (self.min_distance_mm <= distance <= self.max_distance_mm):
                    continue
                index = self._to_bearing_index(point["a_deg"])
                self._distances[index] = distance
                self._updated_at[index] = now
                accepted += 1

        self._points_received += accepted
        self._update_rate(accepted, now)

    def _to_bearing_index(self, raw_angle_deg):
        angle = -raw_angle_deg if self.mirror else raw_angle_deg
        return int(round(angle + self.angle_offset_deg)) % 360

    def _update_rate(self, accepted, now):
        if self._rate_window_start == 0.0:
            self._rate_window_start = now
        self._rate_window_points += accepted
        elapsed = now - self._rate_window_start
        if elapsed >= 0.25:
            # report from a partial window too, so the debug overlay shows a
            # real rate right away instead of 0 for the first second
            self._points_per_second = self._rate_window_points / elapsed
        if elapsed >= 1.0:
            self._rate_window_start = now
            self._rate_window_points = 0

    # ========================================================================
    # READING THE SCAN
    # ========================================================================

    def get_scan(self, max_age_s=None):
        """
        I/O:
            max_age_s: override the staleness cutoff; None uses
                        self.stale_after_s, and math.inf keeps every reading
                        ever seen (useful for a static bench test)
            return: 360-element float array of millimeters, indexed by whole
                    degrees of bearing, np.nan where there is no fresh reading
        """
        with self._lock:
            distances = self._distances.copy()
            updated_at = self._updated_at.copy()

        cutoff = self.stale_after_s if max_age_s is None else max_age_s
        if cutoff is not None and cutoff != math.inf:
            distances[(time.monotonic() - updated_at) > cutoff] = np.nan
        return distances

    def get_distance(self, bearing_deg, window_deg=0.0, max_age_s=None):
        """
        Distance in mm at a bearing, or np.nan if nothing was seen there.
        With window_deg > 0 this returns the CLOSEST reading within
        +/- window_deg, which is what you want for obstacle avoidance - a
        single degree regularly comes back empty.
        """
        if window_deg <= 0:
            return self.get_scan(max_age_s)[int(round(bearing_deg)) % 360]
        distance, _ = self.get_min_distance(bearing_deg - window_deg,
                                            bearing_deg + window_deg, max_age_s)
        return distance

    def get_min_distance(self, start_deg, end_deg, max_age_s=None):
        """
        Closest reading in a sector, sweeping clockwise from start_deg to
        end_deg (so 350 -> 10 is the 20 degrees straddling straight ahead).

        I/O:
            return: (distance_mm, bearing_deg), or (np.nan, None) if the whole
                    sector is empty
        """
        scan = self.get_scan(max_age_s)
        indices = self._sector_indices(start_deg, end_deg)
        sector = scan[indices]
        if np.all(np.isnan(sector)):
            return np.nan, None
        closest = int(np.nanargmin(sector))
        return float(sector[closest]), int(indices[closest])

    def get_points(self, max_age_s=None):
        """List of (bearing_deg, distance_mm) for every direction with data."""
        scan = self.get_scan(max_age_s)
        return [(int(angle), float(scan[angle]))
                for angle in np.flatnonzero(~np.isnan(scan))]

    @staticmethod
    def _sector_indices(start_deg, end_deg):
        start = int(math.floor(start_deg)) % 360
        end = int(math.ceil(end_deg)) % 360
        if start <= end:
            return np.arange(start, end + 1)
        return np.concatenate([np.arange(start, 360), np.arange(0, end + 1)])  # wraps past 0

    # ========================================================================
    # DEBUG VISUALIZATION
    # ========================================================================

    def show_debug(self, window_name="Lidar", max_age_s=None):
        """
        Draws the top-down view into an OpenCV window. No-op unless
        self.debug is True. Call this from your main loop (OpenCV windows
        must be driven from the main thread) followed by cv2.waitKey(1).
        """
        if not self.debug:
            return None
        canvas = self.render_debug(max_age_s)
        cv2.imshow(window_name, canvas)
        return canvas

    def render_debug(self, max_age_s=None):
        """Renders the top-down scan view and returns it as a BGR image."""
        size = CANVAS_PX
        canvas = np.zeros((size, size, 3), dtype=np.uint8)
        origin = (size // 2, size // 2)
        px_per_mm = (size / 2 - CANVAS_MARGIN_PX) / self.render_range_mm

        self._draw_range_rings(canvas, origin, px_per_mm)
        self._draw_axes(canvas, origin)

        scan = self.get_scan(max_age_s)
        for angle in np.flatnonzero(~np.isnan(scan)):
            distance = scan[angle]
            radius_px = distance * px_per_mm
            if radius_px > size / 2 - CANVAS_MARGIN_PX:
                continue
            # forward (0 deg) points up the canvas, bearing increases clockwise
            radians = math.radians(angle)
            x = int(origin[0] + math.sin(radians) * radius_px)
            y = int(origin[1] - math.cos(radians) * radius_px)
            cv2.circle(canvas, (x, y), 2, self._distance_color(distance), -1)

        cv2.circle(canvas, origin, 6, (255, 255, 255), -1)
        self._draw_stats(canvas, scan, max_age_s)
        return canvas

    def debug_text(self, buckets=36, max_age_s=None):
        """
        Same data as render_debug, as a printable block for running over SSH
        with no display. One line per (360 / buckets) degree slice, showing
        the closest reading in that slice in centimeters.
        """
        scan = self.get_scan(max_age_s)
        step = 360 // buckets
        lines = [f"port={self.port}  points={self._points_received}  "
                 f"rate={self._points_per_second:.0f}/s  "
                 f"valid={np.count_nonzero(~np.isnan(scan))}/360"]
        for start in range(0, 360, step):
            sector = scan[start:start + step]
            if np.all(np.isnan(sector)):
                lines.append(f"{start:>4}-{start + step - 1:<4} {'-':>8}")
                continue
            closest_cm = float(np.nanmin(sector)) / 10.0
            bar = "#" * min(40, int(closest_cm / 5))
            lines.append(f"{start:>4}-{start + step - 1:<4} {closest_cm:>7.1f}cm {bar}")
        return "\n".join(lines)

    def _distance_color(self, distance_mm):
        # near -> bright green, far -> dim blue
        t = min(1.0, distance_mm / self.render_range_mm)
        return int(80 + 175 * t), int(255 - 165 * t), 60

    def _draw_range_rings(self, canvas, origin, px_per_mm):
        max_px = canvas.shape[0] / 2 - CANVAS_MARGIN_PX
        # keep roughly RING_TARGET_COUNT rings however far the view is zoomed
        step = max(RING_STEP_MM,
                   int(round(self.render_range_mm / RING_TARGET_COUNT / RING_STEP_MM)) * RING_STEP_MM)
        for distance_mm in range(step, int(self.render_range_mm) + 1, step):
            radius_px = int(distance_mm * px_per_mm)
            if radius_px > max_px:
                break
            cv2.circle(canvas, origin, radius_px, (55, 55, 55), 1)
            # label on the up-right diagonal so it stays clear of the axes
            offset = int(radius_px * 0.7071)
            ImageDrawingUtils.draw_label(canvas, f"{distance_mm // 10}cm",
                                         (origin[0] + offset + 4, origin[1] - offset - 4),
                                         (120, 120, 120), scale=0.4)

    @staticmethod
    def _draw_axes(canvas, origin):
        size = canvas.shape[0]
        cv2.line(canvas, (origin[0], CANVAS_MARGIN_PX), (origin[0], size - CANVAS_MARGIN_PX),
                 (55, 55, 55), 1)
        cv2.line(canvas, (CANVAS_MARGIN_PX, origin[1]), (size - CANVAS_MARGIN_PX, origin[1]),
                 (55, 55, 55), 1)
        ImageDrawingUtils.draw_label(canvas, "FWD 0", (origin[0] + 6, CANVAS_MARGIN_PX + 14),
                                     (200, 200, 200), scale=0.5)
        ImageDrawingUtils.draw_label(canvas, "+90", (size - CANVAS_MARGIN_PX - 40, origin[1] - 8),
                                     (200, 200, 200), scale=0.5)
        ImageDrawingUtils.draw_label(canvas, "-90", (CANVAS_MARGIN_PX + 4, origin[1] - 8),
                                     (200, 200, 200), scale=0.5)

    def _draw_stats(self, canvas, scan, max_age_s):
        valid = np.count_nonzero(~np.isnan(scan))
        lines = [f"{self._points_per_second:.0f} pts/s   {valid}/360 deg   "
                 f"{self._points_received} total"]
        for label, start, end in SUMMARY_SECTORS:
            distance, bearing = self.get_min_distance(start, end, max_age_s)
            if bearing is None:
                lines.append(f"{label:<6} --")
            else:
                lines.append(f"{label:<6} {distance / 10:6.1f}cm @ {bearing:>3}deg")

        for i, line in enumerate(lines):
            ImageDrawingUtils.draw_label(canvas, line, (12, 24 + i * 20),
                                         (255, 255, 255), scale=0.5)

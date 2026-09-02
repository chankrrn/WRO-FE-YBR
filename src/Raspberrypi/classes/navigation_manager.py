"""
Where-am-I-on-the-field, from the lidar and the IMU.

The short answer to "can we predict the robot position with these sensors?" is
yes, and without much ambiguity. The field is a known 3200mm box with a known
1000mm block in the middle, so for any pose there is exactly one 360-degree
range profile the lidar should see. Comparing the profile we actually see
against the profile the map predicts pins the robot down.

The one catch is symmetry: the map looks identical after a 90-degree rotation
about the center, so ranges ALONE leave four candidate poses. That is what the
IMU is for - it says which of the four we are in. After that the lidar does the
precise work, because on the ring the walls are never far away and a 10mm range
error is a 10mm position error.

How it works
------------
Monte Carlo localization (a particle filter):

    predict   move every particle by however far the robot travelled, plus
              noise, and set its heading from the IMU (+ noise)
    correct   raycast the map from every particle, score it against the real
              scan, and resample toward the particles that explain the scan
    estimate  weighted mean of the cloud's DOMINANT cluster, plus a confidence
              from how tight that cluster is, how much of the cloud agrees with
              it, and how well it explains the scan

Heading gets one extra trick. The walls are all axis aligned, so the lidar can
measure the robot's heading modulo 90 degrees far more accurately than a
drifting BNO055 can - see `scan_orientation`. We take the precise angle from
the lidar and only the quadrant from the IMU, which means IMU drift has to
exceed 45 degrees before it can hurt us.

Usage:
    nav = NavigationManager(context.lidar, context.compass, debug=True)
    nav.start()                      # or nav.start(x, y, heading) if known
    ...
    pose = nav.get_pose()            # Pose(x_mm, y_mm, heading_deg, ...)
    if pose.is_reliable:
        print(pose.x_m, pose.y_m, pose.heading)
    nav.show_debug()                 # needs a cv2.waitKey(1) in your loop
"""
import math
import time
from dataclasses import dataclass, replace

import cv2
import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

from classes.block_map import BLOCK_SIZE_MM, BlockMap, camera_offset_behind_lidar
from classes.field_map import FieldMap
from utils.angle_utils import angle_difference, clamp, normalize_angle
from utils.image_drawing_utils import ImageDrawingUtils

# ============================================================================
# Sensor field of view
# ============================================================================
# Behind roughly +/-120 degrees the beam hits the robot's own body, so those
# bearings come back as a constant few centimeters that no pose can explain.
# SENSOR_OUTLIER_WEIGHT means the filter survives them, but they still burn a
# third of the beam budget on nothing and inflate the match error - which is
# what confidence is computed from - so they are cut before anything sees them.
DEFAULT_FOV_DEG = (-120.0, 120.0)

# ============================================================================
# Filter configuration
# ============================================================================
PARTICLE_COUNT = 500
BEAM_COUNT = 48                  # beams scored per update, spread over the scan
MIN_BEAMS = 12                   # fewer than this and the update is skipped

# Half the beams landing within ~SENSOR_SIGMA_MM of the map is a good match.
# Too tight and the filter starves itself on a slightly miscalibrated mount.
SENSOR_SIGMA_MM = 80.0
# Probability mass reserved for "this beam hit something not on the map" -
# a pillar, another robot, the wall's rounded corner. Without it a handful of
# unexplained beams can veto an otherwise perfect pose.
SENSOR_OUTLIER_WEIGHT = 0.15

# Process noise. There are no wheel encoders, so unless a caller reports
# motion the filter only knows "the robot may have moved a bit since last
# time" - this is that "a bit", per second.
DRIFT_SIGMA_MM_S = 220.0
MOTION_NOISE_FRACTION = 0.25     # extra spread proportional to distance moved
HEADING_SIGMA_DEG = 3.0

RESAMPLE_ESS_FRACTION = 0.5      # resample once the effective sample size halves
ROUGHENING_MM = 12.0             # jitter after resampling, kills duplicate clones

# Particles must stay this far off the walls - the robot has a body.
ROBOT_CLEARANCE_MM = 60.0

# ============================================================================
# Confidence
# ============================================================================
CONFIDENCE_SPREAD_MM = 400.0     # cloud spread at which confidence falls to 1/e
CONFIDENCE_MATCH_MM = 250.0      # median beam error at which it falls to 1/e
RELIABLE_CONFIDENCE = 0.35
# Particles within this of the heaviest one count as the same hypothesis.
CLUSTER_RADIUS_MM = 300.0
# Recovery runs off a smoothed confidence, not the instantaneous one. On a
# sparse scan a wrong pose can score well for a frame or two by luck, and a
# plain "low for N seconds" timer gets reset by every one of those flukes -
# so the filter would sit lost forever, occasionally looking fine. Smoothing
# means a spike has to be sustained to count as having found something.
CONFIDENCE_SMOOTHING = 0.25      # weight of the newest reading, per update
RECOVERY_CONFIDENCE = 0.25       # smoothed confidence below which we are lost
RECOVERY_FRACTION = 0.3          # of the cloud, re-scattered per update

# ============================================================================
# Lidar-derived heading
# ============================================================================
# Wall direction is fitted over a window of bearings rather than between
# neighbours: at 1m range two beams 1 degree apart are ~17mm apart, which is
# the same size as the C1's own noise, so a single chord carries no angle
# information at all. Nine degrees of wall is ~150mm and fits cleanly.
ORIENTATION_WINDOW_DEG = 9
ORIENTATION_MIN_POINTS = 6          # valid returns needed inside a window
ORIENTATION_MAX_SPREAD_MM = 350.0   # range spread that still reads as one surface
ORIENTATION_MIN_ANISOTROPY = 0.90   # 1.0 = perfectly straight, drops at corners
MIN_ORIENTATION_WINDOWS = 8
ORIENTATION_MIN_QUALITY = 0.55      # 0..1, how consistently square the scan looks

# ============================================================================
# Debug visualizer
# ============================================================================
CANVAS_PX = 760
CANVAS_MARGIN_PX = 40
COLOR_WALL = (90, 90, 90)
COLOR_BLOCK = (45, 45, 70)
COLOR_PARTICLE = (150, 90, 40)
COLOR_SCAN = (90, 240, 120)
COLOR_ROBOT = (60, 90, 250)
COLOR_TRUTH = (250, 220, 60)

# The auto-refresh window for get_pose(). Anything fresher than this is
# returned as-is instead of burning a raycast per call.
#
# The C1 spins at ~10Hz, so a scan-derived pose can never be fresher than
# 100ms however often this fires; setting it just under that means we consume
# every rotation and no more. Between rotations get_pose() extrapolates with
# the odometry rather than handing back the same stale pose - see
# `_extrapolated`, which is what keeps a 50Hz control loop smooth.
AUTO_UPDATE_AFTER_S = 0.08


@dataclass(frozen=True)
class Pose:
    """
    Robot pose on the field. x/y are millimeters from the center of the field,
    heading is degrees clockwise from +Y (so 0 = facing +Y, 90 = facing +X).

    `confidence` answers "does this pose explain what the lidar sees", which is
    not quite the same as "is this pose correct". The field is 90-degree
    rotationally symmetric, so a pose in the wrong quadrant explains the scan
    perfectly and reports a confidence just as high. Only the IMU tells the
    quadrants apart - with a working compass this cannot happen, and without
    one no amount of lidar will fix it.
    """
    x: float
    y: float
    heading: float
    confidence: float = 0.0
    spread_mm: float = float("inf")
    match_error_mm: float = float("inf")
    timestamp: float = 0.0

    @property
    def x_m(self):
        return self.x / 1000.0

    @property
    def y_m(self):
        return self.y / 1000.0

    @property
    def is_reliable(self):
        return self.confidence >= RELIABLE_CONFIDENCE

    def as_tuple(self):
        return self.x, self.y, self.heading

    def __str__(self):
        return (f"({self.x / 10:.1f}cm, {self.y / 10:.1f}cm) @ {self.heading:.1f}deg "
                f"conf={self.confidence:.2f}")


class NavigationManager:
    """
    Particle-filter localization on a FieldMap, fed by a LidarManager and a
    CompassManager.

    Neither sensor is mandatory. Without the compass the filter still locks on
    to a pose, but which of the four rotationally identical quadrants it picks
    is arbitrary. Without the lidar there is nothing to localize against and
    every pose comes back with confidence 0, which is the honest answer.

    The lidar is only ever asked for `get_scan()`, so anything with that method
    (see the simulator in test_navigation.py) can drive this.
    """

    def __init__(self, lidar=None, compass=None, field_map=None, debug=False,
                 particle_count=PARTICLE_COUNT, beam_count=BEAM_COUNT,
                 lidar_offset_mm=(0.0, 0.0), compass_sign=1.0,
                 use_lidar_heading=True, sensor_sigma_mm=SENSOR_SIGMA_MM,
                 fov_deg=DEFAULT_FOV_DEG, block_map=None, seed=None):
        """
        I/O:
            lidar: LidarManager (or anything exposing get_scan())
            compass: CompassManager, or None to run heading-blind
            lidar_offset_mm: (forward, right) of the sensor from the robot's
                             center, in the robot's own frame
            compass_sign: +1 if the compass counts up clockwise, -1 if it
                          counts up counter-clockwise. Get this wrong and the
                          pose will spin the wrong way - test_navigation.py
                          --hardware prints what it sees so you can check.
            use_lidar_heading: refine the IMU heading with the wall angle
            fov_deg: (start, end) bearings to keep, sweeping clockwise from
                     start to end. The default cuts the rear arc where the
                     beam hits the robot itself; None keeps all 360.
            block_map: BlockMap for the red/green pillars. One is made if you
                       do not pass one, so nav.blocks is always there - it just
                       stays empty until something calls observe_blocks(). The
                       one made here puts the lens CAMERA_BEHIND_LIDAR_MM
                       behind the lidar; pass your own BlockMap if the camera
                       is somewhere else on the robot.
        """
        self.lidar = lidar
        self.compass = compass
        self.map = field_map or FieldMap()
        self.debug = debug
        self.particle_count = int(particle_count)
        self.beam_count = int(beam_count)
        self.lidar_offset_mm = tuple(float(v) for v in lidar_offset_mm)
        self.compass_sign = float(compass_sign)
        self.use_lidar_heading = use_lidar_heading
        self.sensor_sigma_mm = float(sensor_sigma_mm)
        self.fov_deg = tuple(float(v) for v in fov_deg) if fov_deg else None
        self._fov_mask = self._build_fov_mask(self.fov_deg)
        # The camera hangs off the same mast as the lidar, so its offset is
        # derived from lidar_offset_mm rather than configured separately -
        # move the mast and both sensors follow.
        self.blocks = block_map or BlockMap(
            field_map=self.map, debug=debug,
            camera_offset_mm=camera_offset_behind_lidar(self.lidar_offset_mm))
        # Last raw camera frame, (detections, monotonic time) - see
        # observe_blocks. None until the camera has produced one.
        self.last_detections = None

        self._rng = np.random.default_rng(seed)

        # Particle state: positions (N, 2) in mm, headings (N,) in degrees,
        # weights (N,) summing to 1.
        self._xy = np.zeros((self.particle_count, 2))
        self._heading = np.zeros(self.particle_count)
        self._weights = np.full(self.particle_count, 1.0 / self.particle_count)

        self._pose = None
        self._started = False
        self._last_update = 0.0
        self._pending_mm = 0.0        # odometry reported since the last update
        self._pending_turn_deg = 0.0
        self._smoothed_confidence = 1.0

        # Kept from the last update purely so the debug view and the caller can
        # see what the filter was actually looking at.
        self._last_scan = None
        self._last_bearings = None
        self._last_measured = None
        self._last_expected = None
        self._heading_source = "none"
        self._orientation_quality = 0.0
        self._update_ms = 0.0
        self._updates = 0

        # Field heading = compass_sign * raw + this. Set by set_start_heading().
        self._compass_offset = None

    # ========================================================================
    # LIFECYCLE
    # ========================================================================

    def start(self, x_mm=None, y_mm=None, heading_deg=None, zones=None):
        """
        Seeds the particle cloud.

        With no arguments it scatters particles over the whole drivable ring
        and lets the lidar find the robot (a second or two of scans), which is
        what you want when you do not trust the exact placement.

        `zones` narrows that search to a list of rectangles - pass
        FieldMap.start_zones() when the rules say the robot can only be set
        down in certain cells. Fewer places to look means a faster and more
        reliable lock, and it costs nothing if the robot really is there.

        Passing a pose seeds a tight cloud around it instead AND anchors the
        compass, so the quadrant is right from the first update.
        """
        if heading_deg is not None:
            self.set_start_heading(heading_deg)

        if x_mm is None or y_mm is None:
            self._scatter(np.arange(self.particle_count), zones=zones)
        else:
            self._xy[:] = (x_mm, y_mm)
            self._xy += self._rng.normal(0.0, 60.0, self._xy.shape)
            base = heading_deg if heading_deg is not None else (self.field_heading() or 0.0)
            self._heading[:] = normalize_angle(base + self._rng.normal(0.0, 5.0,
                                                                      self.particle_count))
        self._weights[:] = 1.0 / self.particle_count

        self._started = True
        self._last_update = time.monotonic()
        self._pending_mm = 0.0
        self._pending_turn_deg = 0.0
        self._smoothed_confidence = 1.0
        self._pose = self._estimate()
        if self.debug:
            where = "global search" if x_mm is None else f"seeded at {self._pose}"
            print(f"[nav] started, {self.particle_count} particles, {where}")
        return self

    def stop(self):
        self._started = False

    @property
    def is_started(self):
        return self._started

    @property
    def is_available(self):
        return self.lidar is not None

    # ========================================================================
    # FIELD OF VIEW
    # ========================================================================

    @staticmethod
    def _build_fov_mask(fov_deg):
        """
        360-element bool mask of the bearings we are allowed to believe, or
        None for "all of them". Sweeps clockwise from start to end, so
        (-120, 120) keeps the front 240 degrees and (170, 190) would keep a
        20-degree slice out the back.
        """
        if fov_deg is None:
            return None
        start, end = fov_deg
        if end - start >= 360.0:
            return None
        bearings = np.arange(360.0)
        # Distance travelled clockwise from start to each bearing.
        return ((bearings - start) % 360.0) <= ((end - start) % 360.0)

    def set_fov(self, fov_deg):
        """
        Changes the accepted bearing window at runtime (None = all 360).

        The lidar's FOV only, deliberately: this used to carry a pasted copy
        of __init__'s block-map construction, which referenced names that do
        not exist in this scope (it would have raised NameError on the first
        call) and, had it run, would have thrown away every pillar mapped so
        far. Where the camera looks has nothing to do with which bearings the
        lidar is allowed to believe.
        """
        self.fov_deg = tuple(float(v) for v in fov_deg) if fov_deg else None
        self._fov_mask = self._build_fov_mask(self.fov_deg)

    def crop_scan(self, scan):
        """
        Blanks every bearing outside the FOV. Everything downstream - beam
        selection, the wall-angle estimate and the debug overlay - runs on the
        cropped scan, so the robot's own body can never be mistaken for a wall.
        """
        if scan is None or self._fov_mask is None:
            return scan
        cropped = np.array(scan, dtype=float, copy=True)
        cropped[~self._fov_mask] = np.nan
        return cropped

    # ========================================================================
    # HEADING
    # ========================================================================

    @property
    def compass_anchored(self):
        """
        True when the compass has been told which way is which on the field.

        Until it has, field_heading() returns the IMU's RAW heading, whose
        zero is wherever the sensor happened to boot - so the quadrant it
        picks is arbitrary but repeatable, which is worse than random. See
        set_start_heading.
        """
        return self.compass is not None and self._compass_offset is not None

    def set_start_heading(self, field_heading_deg):
        """
        Anchors the compass: says "whatever the IMU reads right now, the robot
        is really pointing `field_heading_deg` on the field". Call this on the
        start tile, where you know which way the robot faces.

        Returns the offset, or None if the compass could not be read (in which
        case the filter falls back to lidar-only heading).
        """
        raw = self._raw_heading()
        if raw is None:
            return None
        self._compass_offset = normalize_angle(field_heading_deg - self.compass_sign * raw)
        if self.debug:
            print(f"[nav] compass anchored: raw {raw:.1f} -> field {field_heading_deg:.1f}")
        return self._compass_offset

    def field_heading(self):
        """The compass reading converted to field degrees, or None."""
        raw = self._raw_heading()
        if raw is None:
            return None
        offset = self._compass_offset if self._compass_offset is not None else 0.0
        return normalize_angle(self.compass_sign * raw + offset)

    def _raw_heading(self):
        if self.compass is None:
            return None
        try:
            return self.compass.heading()
        except Exception:
            return None

    def scan_orientation(self, scan):
        """
        Heading modulo 90 degrees, measured from the walls themselves.

        Every surface in the field runs along a field axis, so a straight run
        of lidar returns lies along a field axis too - seen from the robot,
        rotated by exactly the robot's heading. So:

            1. slide a window along the scan and fit a line through each
               windowful of points (the principal axis of their scatter)
            2. throw away windows that are not convincingly straight, which is
               how corners and window-fuls of two different walls get dropped
            3. multiply every surviving direction by four. The four wall
               directions are 90 degrees apart, so x4 lands them all on the
               same angle and they can be averaged as one population
            4. divide the average back by four

        The result is the heading modulo 90, typically to a fraction of a
        degree - far better than a BNO055 that has been spun around a few
        corners. `quality` is the length of the averaged unit vector: 1.0 means
        every window agreed, and anything below ORIENTATION_MIN_QUALITY means
        the scan was too round or too sparse to call.

        I/O:
            return: (heading_mod_90_deg, quality 0..1), or (None, quality) when
                    it cannot be measured
        """
        scan = np.asarray(scan, dtype=float)
        angles = np.radians(np.arange(360.0))
        px = scan * np.sin(angles)       # robot frame: +x right, +y forward
        py = scan * np.cos(angles)

        # Wrap the first window's worth onto the end so a window can straddle
        # bearing 0 like any other.
        window = ORIENTATION_WINDOW_DEG
        pad = window - 1
        windows_x = sliding_window_view(np.concatenate([px, px[:pad]]), window)
        windows_y = sliding_window_view(np.concatenate([py, py[:pad]]), window)
        windows_r = sliding_window_view(np.concatenate([scan, scan[:pad]]), window)

        populated = np.count_nonzero(~np.isnan(windows_r), axis=1) >= ORIENTATION_MIN_POINTS
        if np.count_nonzero(populated) < MIN_ORIENTATION_WINDOWS:
            return None, 0.0
        windows_x = windows_x[populated]
        windows_y = windows_y[populated]
        windows_r = windows_r[populated]

        offset_x = windows_x - np.nanmean(windows_x, axis=1, keepdims=True)
        offset_y = windows_y - np.nanmean(windows_y, axis=1, keepdims=True)
        var_x = np.nanmean(offset_x * offset_x, axis=1)
        var_y = np.nanmean(offset_y * offset_y, axis=1)
        covariance = np.nanmean(offset_x * offset_y, axis=1)

        # Eigenvalue gap over trace: 1.0 for points on a perfect line, and it
        # collapses toward 0 for a window that turned a corner.
        total = var_x + var_y
        gap = np.hypot(var_x - var_y, 2.0 * covariance)
        with np.errstate(divide="ignore", invalid="ignore"):
            anisotropy = np.where(total > 0.0, gap / total, 0.0)
        spread = np.nanmax(windows_r, axis=1) - np.nanmin(windows_r, axis=1)

        keep = ((anisotropy >= ORIENTATION_MIN_ANISOTROPY)
                & (spread <= ORIENTATION_MAX_SPREAD_MM)
                & (total > 1.0))
        if np.count_nonzero(keep) < MIN_ORIENTATION_WINDOWS:
            return None, 0.0

        directions = 0.5 * np.arctan2(2.0 * covariance[keep], var_x[keep] - var_y[keep])
        weights = gap[keep]          # long, straight windows outvote short ones
        folded = np.sum(weights * np.exp(4j * directions))
        quality = float(np.abs(folded) / np.sum(weights))
        if quality < ORIENTATION_MIN_QUALITY:
            return None, quality
        return float(np.degrees(np.angle(folded)) / 4.0) % 90.0, quality

    def _resolve_heading(self, scan):
        """
        Best heading estimate for this tick: the lidar's precise angle snapped
        into the quadrant the IMU (or, failing that, the previous pose) says we
        are in. Returns None when nothing can say which way we point.
        """
        coarse = self.field_heading()
        if coarse is None and self._pose is not None:
            coarse = self._pose.heading
        self._heading_source = "compass" if self.field_heading() is not None else "filter"

        if not self.use_lidar_heading or scan is None:
            return coarse

        fine, quality = self.scan_orientation(scan)
        self._orientation_quality = quality
        if fine is None:
            return coarse
        if coarse is None:
            # Nothing to break the 90-degree symmetry, so pick a branch and
            # stay consistent with it. Which of the four it is, is a coin toss.
            self._heading_source = "lidar (quadrant arbitrary)"
            return fine

        # Keep the IMU's quadrant, take the lidar's angle within it: fold the
        # disagreement into +/-45 degrees, which is all `fine` can resolve.
        delta = (angle_difference(fine, coarse) + 45.0) % 90.0 - 45.0
        self._heading_source = f"lidar+{self._heading_source}"
        return normalize_angle(coarse + delta)

    # ========================================================================
    # FILTER
    # ========================================================================

    def report_motion(self, distance_mm, turn_deg=0.0):
        """
        Odometry hook: how far the robot drove and how far it turned since the
        last call (negative distance for reverse).

        This does two jobs. It gives the filter a motion model to predict with
        instead of pure diffusion, and - more importantly for the control loop -
        it is what lets get_pose() extrapolate between scans. A caller that
        reports motion every tick gets a pose that moves every tick, even
        though the lidar only produces a new one ten times a second.

        Both are cheap to supply: `distance_mm` is speed x dt, and `turn_deg`
        is the bicycle model's yaw change, degrees(distance / wheelbase *
        tan(road_wheel_angle)).
        """
        self._pending_mm += float(distance_mm)
        self._pending_turn_deg += float(turn_deg)

    def update(self, scan=None, moved_mm=None):
        """
        One filter tick: predict, correct, resample, estimate.

        I/O:
            scan: 360-element array from LidarManager.get_scan(); read from
                  self.lidar when omitted
            moved_mm: distance driven since the last update, overriding
                      whatever report_motion() accumulated
            return: the new Pose (confidence 0 if there was nothing to score)
        """
        if not self._started:
            self.start()

        started_at = time.monotonic()
        dt = max(1e-3, started_at - self._last_update)
        self._last_update = started_at

        if scan is None and self.lidar is not None:
            scan = self.lidar.get_scan()
        scan = self.crop_scan(scan)
        self._last_scan = scan

        distance = self._pending_mm if moved_mm is None else float(moved_mm)
        self._pending_mm = 0.0
        self._pending_turn_deg = 0.0

        heading = self._resolve_heading(scan)
        self._predict(dt, distance, heading)
        scored = self._correct(scan)

        if scored:
            if self._effective_sample_size() < RESAMPLE_ESS_FRACTION * self.particle_count:
                self._resample()
            self._pose = self._estimate()
            self._maybe_recover()
        else:
            # No usable scan: the cloud has been spread by _predict and there
            # is nothing to pull it back, so report the drift honestly.
            self._pose = self._estimate()

        self._updates += 1
        self._update_ms = (time.monotonic() - started_at) * 1000.0
        return self._pose

    def _predict(self, dt, distance_mm, heading_deg):
        count = self.particle_count

        if heading_deg is not None:
            self._heading = normalize_angle(
                heading_deg + self._rng.normal(0.0, HEADING_SIGMA_DEG, count))
        else:
            self._heading = normalize_angle(
                self._heading + self._rng.normal(0.0, HEADING_SIGMA_DEG * 2, count))

        if distance_mm:
            radians = np.radians(self._heading)
            travelled = distance_mm + self._rng.normal(
                0.0, abs(distance_mm) * MOTION_NOISE_FRACTION + 5.0, count)
            self._xy[:, 0] += travelled * np.sin(radians)
            self._xy[:, 1] += travelled * np.cos(radians)
            spread = abs(distance_mm) * MOTION_NOISE_FRACTION
        else:
            spread = DRIFT_SIGMA_MM_S * dt

        self._xy += self._rng.normal(0.0, max(spread, 5.0), self._xy.shape)
        np.clip(self._xy, -self.map.outer, self.map.outer, out=self._xy)

    def _correct(self, scan):
        """Reweights the cloud against the scan. False if it could not."""
        bearings, measured = self._select_beams(scan)
        self._last_bearings, self._last_measured = bearings, measured
        if bearings is None:
            return False

        expected = self._expected_ranges(self._xy, self._heading, bearings)
        error = measured[None, :] - expected

        # Gaussian around the map prediction, on a floor of "unmapped object".
        # Summing logs rather than multiplying keeps 48 beams from underflowing.
        likelihood = np.exp(-0.5 * (error / self.sensor_sigma_mm) ** 2)
        log_weights = np.sum(np.log((1.0 - SENSOR_OUTLIER_WEIGHT) * likelihood
                                    + SENSOR_OUTLIER_WEIGHT), axis=1)

        # A particle inside a wall explains nothing, whatever its beams say.
        log_weights[~self.map.contains(self._xy[:, 0], self._xy[:, 1],
                                       ROBOT_CLEARANCE_MM)] = -np.inf

        weights = np.exp(log_weights - np.max(log_weights))
        total = weights.sum()
        if not np.isfinite(total) or total <= 0.0:
            # Every particle is impossible - the cloud is somewhere it should
            # not be, so start the search over rather than dividing by zero.
            self._scatter(np.arange(self.particle_count))
            self._weights[:] = 1.0 / self.particle_count
            return False

        self._weights = weights / total
        self._last_expected = expected
        return True

    def _select_beams(self, scan):
        """
        Picks up to `beam_count` directions that actually have data, spread
        evenly around the scan so no one wall dominates the score.
        """
        if scan is None:
            return None, None
        scan = np.asarray(scan, dtype=float)
        valid = np.flatnonzero(~np.isnan(scan))
        valid = valid[scan[valid] <= self.map.diagonal_mm]
        if valid.size < MIN_BEAMS:
            return None, None
        if valid.size > self.beam_count:
            picks = np.linspace(0, valid.size - 1, self.beam_count).astype(int)
            valid = valid[np.unique(picks)]
        return valid.astype(float), scan[valid.astype(int)]

    def _expected_ranges(self, xy, headings, bearings):
        """
        Map ranges each particle's lidar should report, shape (N, len(bearings)).
        The sensor sits at `lidar_offset_mm` in the robot frame, so the rays
        start there and not at the particle itself.
        """
        forward, right = self.lidar_offset_mm
        radians = np.radians(headings)
        sensor_x = xy[:, 0] + forward * np.sin(radians) + right * np.cos(radians)
        sensor_y = xy[:, 1] + forward * np.cos(radians) - right * np.sin(radians)

        world_angles = np.radians(headings[:, None] + bearings[None, :])
        return self.map.raycast(sensor_x[:, None], sensor_y[:, None], world_angles)

    def _effective_sample_size(self):
        return 1.0 / np.sum(self._weights ** 2)

    def _resample(self):
        """
        Systematic resampling: one evenly spaced draw per particle from a
        single random offset. Lower variance than drawing N times
        independently, and it never drops a heavy particle.
        """
        positions = (self._rng.random() + np.arange(self.particle_count)) / self.particle_count
        picks = np.searchsorted(np.cumsum(self._weights), positions)
        picks = np.clip(picks, 0, self.particle_count - 1)

        self._xy = self._xy[picks] + self._rng.normal(0.0, ROUGHENING_MM, self._xy.shape)
        self._heading = self._heading[picks]
        self._weights[:] = 1.0 / self.particle_count

    def _scatter(self, indices, zones=None):
        """
        Drops the given particles anywhere on the drivable ring, or anywhere
        inside `zones` if the search is being restricted to legal start cells.
        """
        count = len(indices)
        if count == 0:
            return
        # Rejection sampling - even the zone case is mostly-drivable area, so
        # this converges immediately and beats sampling the region directly.
        low, high = self._sample_bounds(zones)
        points = np.empty((0, 2))
        while len(points) < count:
            batch = self._rng.uniform(low, high, (count * 3, 2))
            keep = self.map.contains(batch[:, 0], batch[:, 1], ROBOT_CLEARANCE_MM)
            if zones is not None:
                keep &= self._inside_zones(batch, zones)
            points = np.vstack([points, batch[keep]])
        self._xy[indices] = points[:count]

        heading = self.field_heading()
        if heading is None:
            self._heading[indices] = self._rng.uniform(0.0, 360.0, count)
        else:
            self._heading[indices] = normalize_angle(heading + self._rng.normal(0.0, 5.0, count))

    def _sample_bounds(self, zones):
        """Bounding box to draw scatter candidates from."""
        if not zones:
            return -self.map.outer, self.map.outer
        lows = np.array([zone[-2] for zone in zones], dtype=float)
        highs = np.array([zone[-1] for zone in zones], dtype=float)
        return np.min(lows, axis=0), np.max(highs, axis=0)

    @staticmethod
    def _inside_zones(points, zones):
        """
        Mask of the points falling in any zone. Zones are (..., low, high) so
        FieldMap.start_zones()' (name, low, high) tuples can be passed
        straight through without unpacking them first.
        """
        inside = np.zeros(len(points), dtype=bool)
        for zone in zones:
            low, high = zone[-2], zone[-1]
            inside |= ((points[:, 0] >= low[0]) & (points[:, 0] <= high[0])
                       & (points[:, 1] >= low[1]) & (points[:, 1] <= high[1]))
        return inside

    def _maybe_recover(self):
        """
        Scatters a slice of the cloud while we are lost - the robot got picked
        up and moved, or the filter locked on to the wrong quadrant and the
        walls have finally disagreed with it.

        The scatter repeats every update until confidence comes back, because
        one round of random particles rarely lands in the right place: each
        round is a fresh lottery over the ring, and the resampler keeps any
        ticket that wins. Smoothing the confidence first (see
        CONFIDENCE_SMOOTHING) is what stops a lucky frame from calling off the
        search, and equally stops one bad scan from starting it.
        """
        self._smoothed_confidence = ((1.0 - CONFIDENCE_SMOOTHING) * self._smoothed_confidence
                                     + CONFIDENCE_SMOOTHING * self._pose.confidence)
        if self._smoothed_confidence >= RECOVERY_CONFIDENCE:
            return

        count = int(self.particle_count * RECOVERY_FRACTION)
        self._scatter(self._rng.choice(self.particle_count, count, replace=False))
        if self.debug and self._updates % 10 == 0:
            print(f"[nav] lost (smoothed confidence {self._smoothed_confidence:.2f}), "
                  f"re-scattering {count} particles per update")

    # ========================================================================
    # ESTIMATE
    # ========================================================================

    def _estimate(self):
        """
        The pose of the DOMINANT cluster, not the mean of the whole cloud.

        During a cold start or a recovery the cloud is genuinely multi-modal -
        two or more places on the field explain the scan equally well - and the
        mean of two modes is a pose in neither of them, usually straight
        through the middle of the map. So: take the heaviest particle, keep
        everything within CLUSTER_RADIUS_MM of it, and average that. Once the
        filter has converged there is only one mode and this is just the mean.

        How much of the cloud's weight that cluster holds is folded into the
        confidence, which is what makes an unresolved two-way tie read as
        uncertain rather than as a confident answer in the wrong place.
        """
        anchor = self._xy[np.argmax(self._weights)]
        in_cluster = np.sum((self._xy - anchor) ** 2, axis=1) <= CLUSTER_RADIUS_MM ** 2
        cluster_weight = float(self._weights[in_cluster].sum())
        if cluster_weight <= 0.0:            # every weight underflowed to zero
            in_cluster = np.ones(self.particle_count, dtype=bool)
            cluster_weight = 1.0

        points = self._xy[in_cluster]
        weights = self._weights[in_cluster] / cluster_weight
        mean = np.average(points, axis=0, weights=weights)

        radians = np.radians(self._heading[in_cluster])
        heading = math.degrees(math.atan2(np.average(np.sin(radians), weights=weights),
                                          np.average(np.cos(radians), weights=weights)))

        variance = np.average(np.sum((points - mean) ** 2, axis=1), weights=weights)
        spread = float(math.sqrt(max(0.0, variance)))
        match = self._match_error(mean, heading)

        if math.isfinite(match):
            confidence = (math.exp(-spread / CONFIDENCE_SPREAD_MM)
                          * math.exp(-match / CONFIDENCE_MATCH_MM)
                          * cluster_weight)
        else:
            confidence = 0.0

        return Pose(x=float(mean[0]), y=float(mean[1]),
                    heading=normalize_angle(heading),
                    confidence=float(clamp(confidence, 0.0, 1.0)),
                    spread_mm=spread, match_error_mm=match,
                    timestamp=time.monotonic())

    def _match_error(self, mean_xy, heading):
        """
        Median |measured - expected| at the estimated pose. This is the number
        that catches a filter that has confidently converged on the wrong pose:
        the cloud is tight, but the walls are not where it thinks they are.
        """
        if self._last_bearings is None or self._last_measured is None:
            return float("inf")
        expected = self._expected_ranges(np.asarray([mean_xy]),
                                         np.asarray([heading]), self._last_bearings)
        return float(np.median(np.abs(self._last_measured - expected[0])))

    # ========================================================================
    # PUBLIC READOUT
    # ========================================================================

    def get_pose(self, max_age_s=AUTO_UPDATE_AFTER_S, extrapolate=True):
        """
        The robot's pose on the field, right now.

        Runs a filter update first if the last one is older than `max_age_s`,
        so a control loop can just call this and never think about the filter.

        The returned pose is EXTRAPOLATED to the present with whatever odometry
        has been reported since the last update. That matters more than it
        sounds: the lidar produces a pose ten times a second, but a control
        loop wants one every tick, and handing back the same stale pose for
        eight ticks in a row makes the steering move in steps - it corrects,
        waits, corrects harder. See `_extrapolated`.

        I/O:
            max_age_s: None to skip the auto-update and read the cache
            extrapolate: False for the raw filter output, which is what the
                         debug view and the block mapper want - they should
                         show and use what was actually measured
            return: Pose. Always non-None once start() has run; check
                    pose.is_reliable before steering on it.
        """
        if not self._started:
            self.start()
        if max_age_s is not None and (self._pose is None
                                      or time.monotonic() - self._last_update > max_age_s):
            self.update()
        return self._extrapolated() if extrapolate else self._pose

    def _extrapolated(self):
        """
        The last filter estimate carried forward by the odometry reported
        since it was taken.

        Dead reckoning is only trusted for the ~100ms between scans, so it
        cannot drift anywhere: the next update replaces it outright. The
        travel is integrated about the MIDPOINT of the turn rather than the
        start of it, which is what keeps a curve from being cut short.

        Confidence is passed through untouched - extrapolating does not make
        the pose less believable over one lidar rotation, and callers use that
        number to decide whether to trust the whole estimate.
        """
        if self._pose is None:
            return None
        if not self._pending_mm and not self._pending_turn_deg:
            return self._pose

        midpoint = math.radians(self._pose.heading + self._pending_turn_deg / 2.0)
        return replace(
            self._pose,
            x=self._pose.x + self._pending_mm * math.sin(midpoint),
            y=self._pose.y + self._pending_mm * math.cos(midpoint),
            heading=normalize_angle(self._pose.heading + self._pending_turn_deg))

    def observe_blocks(self, detections):
        """
        Folds a frame of ObjectSolver detections into the field-frame block map.

        The detections are relative to the camera, so they are only as good as
        the pose they are placed with - BlockMap drops the whole frame if the
        pose is not reliable rather than scattering pillars across the field.

        I/O:
            detections: list of DetectedObject from ObjectSolver.detect()
            return: the Blocks hit or created by this frame

        Usage, from a task's camera tick:
            objects = context.object_solver.detect(context.camera.hsv_image)
            context.nav.observe_blocks(objects)
            ...
            for block in context.nav.blocks.confirmed():
                plan_around(block.x, block.y)
        """
        # Keep the frame as the camera reported it, before the pose is folded
        # in. The final round re-ranges these against the lidar and never
        # places them through the filter at all, so what it needs is the raw
        # (colour, bearing) pair - which the field-frame Blocks no longer
        # carry. One tuple assignment, so the control loop can read it without
        # a lock; a torn read is impossible and a stale one is a frame old.
        self.last_detections = (detections, time.monotonic())
        # max_age_s=None deliberately: this is called from whichever thread
        # runs the camera, and the default would let it trigger a full filter
        # update - predict/correct/resample, all mutating the particle arrays -
        # concurrently with the control loop doing the same thing. The block
        # map wants the last MEASURED pose anyway, not a freshly computed one.
        return self.blocks.observe(
            self.get_pose(max_age_s=None, extrapolate=False), detections)

    def get_position_m(self):
        """(x, y) in meters from the field center - the short form of get_pose()."""
        pose = self.get_pose()
        return pose.x_m, pose.y_m

    def distance_to_wall_mm(self, bearing_deg=0.0):
        """
        How far the MAP says it is to a wall along `bearing_deg` from the
        robot's nose. Useful as a sanity check against the live lidar reading,
        and as a lookahead the lidar cannot give (through a pillar, say).
        """
        pose = self.get_pose()
        angle = math.radians(pose.heading + bearing_deg)
        return float(self.map.raycast(pose.x, pose.y, np.asarray(angle)))

    def block_accuracy(self, truth_blocks):
        """
        Scores the block map against where the pillars really are. Simulator
        only - on the real field nobody knows the truth, which is the whole
        reason the map exists.

        I/O:
            truth_blocks: [(color, x_mm, y_mm), ...]
            return: (how many were found, mean position error in mm)
        """
        errors = []
        for color, x, y in truth_blocks:
            match = self.blocks.nearest(x, y, color=color)
            if match is not None:
                errors.append(math.hypot(match.x - x, match.y - y))
        if not errors:
            return 0, float("inf")
        return len(errors), float(np.mean(errors))

    def status_line(self):
        pose = self.get_pose(max_age_s=None)
        if pose is None:
            return "[nav] not started"
        return (f"[nav] {pose}  spread={pose.spread_mm:.0f}mm "
                f"match={pose.match_error_mm:.0f}mm heading={self._heading_source} "
                f"({self._update_ms:.1f}ms)")

    # ========================================================================
    # DEBUG VISUALIZATION
    # ========================================================================

    def show_debug(self, window_name="Navigation", truth=None, truth_blocks=None):
        """
        Draws the field view into an OpenCV window. No-op unless self.debug is
        set. Call from the main thread and follow it with cv2.waitKey(1).
        """
        if not self.debug:
            return None
        canvas = self.render_debug(truth=truth, truth_blocks=truth_blocks)
        cv2.imshow(window_name, canvas)
        return canvas

    def render_debug(self, truth=None, truth_blocks=None):
        """
        Top-down field view: walls, particle cloud, the live scan projected
        through the estimated pose, and the estimate itself.

        The scan is the thing to watch. When localization is right, the green
        points sit ON the drawn walls. When they sit beside them, the pose is
        off by exactly that much.

        Red/green pillars from the camera are drawn as life-size 5cm squares
        wherever BlockMap has pinned them, with the camera's cone showing what
        it can currently see. Unconfirmed blocks are hollow and grey.

        I/O:
            truth: optional (x_mm, y_mm, heading_deg) ground truth to overlay,
                   which only a simulator can supply
            truth_blocks: optional [(color, x_mm, y_mm), ...] marking where the
                          pillars really are, also simulator-only
            return: BGR image
        """
        canvas = np.full((CANVAS_PX, CANVAS_PX, 3), 22, dtype=np.uint8)
        scale = (CANVAS_PX - 2 * CANVAS_MARGIN_PX) / self.map.field_size_mm
        center = CANVAS_PX // 2

        def to_px(x, y):
            return int(round(center + x * scale)), int(round(center - y * scale))

        self._draw_field(canvas, to_px)

        for point in self._xy:
            cv2.circle(canvas, to_px(point[0], point[1]), 1, COLOR_PARTICLE, -1)

        pose = self._pose
        if pose is not None:
            self._draw_scan(canvas, to_px, pose)
            self.blocks.draw_camera_cone(canvas, to_px, pose)
            self._draw_robot(canvas, to_px, scale, pose, COLOR_ROBOT)
        # Blocks go on top of the scan: they are 5cm squares and the scan dots
        # are 2px, so drawn underneath they would be all but invisible.
        self.blocks.draw(canvas, to_px, scale)
        if truth_blocks:
            self._draw_truth_blocks(canvas, to_px, truth_blocks)
        if truth is not None:
            self._draw_robot(canvas, to_px, scale,
                             Pose(truth[0], truth[1], truth[2]), COLOR_TRUTH)

        self._draw_stats(canvas, pose, truth, truth_blocks)
        return canvas

    @staticmethod
    def _draw_truth_blocks(canvas, to_px, truth_blocks):
        """Hollow outlines of where the pillars really are, for the simulator."""
        half = BLOCK_SIZE_MM / 2.0
        for _, x, y in truth_blocks:
            cv2.rectangle(canvas, to_px(x - half, y + half), to_px(x + half, y - half),
                          COLOR_TRUTH, 1)

    def _draw_field(self, canvas, to_px):
        (outer_min, outer_max), (inner_min, inner_max) = self.map.rectangles()
        cv2.rectangle(canvas, to_px(*outer_min), to_px(*outer_max), COLOR_WALL, 2)
        cv2.rectangle(canvas, to_px(*inner_min), to_px(*inner_max), COLOR_BLOCK, -1)
        cv2.rectangle(canvas, to_px(*inner_min), to_px(*inner_max), COLOR_WALL, 2)

        # Half-meter grid, so distances are readable straight off the picture.
        step = 500
        for value in range(-1500, 1501, step):
            if value == 0:
                continue
            cv2.line(canvas, to_px(value, -self.map.outer), to_px(value, self.map.outer),
                     (38, 38, 38), 1)
            cv2.line(canvas, to_px(-self.map.outer, value), to_px(self.map.outer, value),
                     (38, 38, 38), 1)
        cv2.line(canvas, to_px(0, -self.map.outer), to_px(0, self.map.outer), (55, 55, 55), 1)
        cv2.line(canvas, to_px(-self.map.outer, 0), to_px(self.map.outer, 0), (55, 55, 55), 1)
        ImageDrawingUtils.draw_label(canvas, "+Y", to_px(0, self.map.outer + 90),
                                     (140, 140, 140), scale=0.45)
        ImageDrawingUtils.draw_label(canvas, "+X", to_px(self.map.outer + 60, 90),
                                     (140, 140, 140), scale=0.45)

    def _draw_scan(self, canvas, to_px, pose):
        if self._last_scan is None:
            return
        scan = np.asarray(self._last_scan, dtype=float)
        indices = np.flatnonzero(~np.isnan(scan))
        if indices.size == 0:
            return

        forward, right = self.lidar_offset_mm
        heading = math.radians(pose.heading)
        origin_x = pose.x + forward * math.sin(heading) + right * math.cos(heading)
        origin_y = pose.y + forward * math.cos(heading) - right * math.sin(heading)

        world = np.radians(pose.heading + indices.astype(float))
        xs = origin_x + scan[indices] * np.sin(world)
        ys = origin_y + scan[indices] * np.cos(world)
        for x, y in zip(xs, ys):
            if abs(x) > self.map.outer + 200 or abs(y) > self.map.outer + 200:
                continue
            cv2.circle(canvas, to_px(x, y), 2, COLOR_SCAN, -1)

    @staticmethod
    def _draw_robot(canvas, to_px, scale, pose, color):
        body_px = max(5, int(90 * scale))
        cv2.circle(canvas, to_px(pose.x, pose.y), body_px, color, 2)
        radians = math.radians(pose.heading)
        nose = (pose.x + 260 * math.sin(radians), pose.y + 260 * math.cos(radians))
        cv2.arrowedLine(canvas, to_px(pose.x, pose.y), to_px(*nose), color, 2, tipLength=0.3)

    def _draw_stats(self, canvas, pose, truth, truth_blocks=None):
        lines = []
        if pose is None:
            lines.append("no estimate yet")
        else:
            lines.append(f"x {pose.x / 10:+7.1f}cm   y {pose.y / 10:+7.1f}cm   "
                         f"hdg {pose.heading:6.1f}deg")
            lines.append(f"confidence {pose.confidence:.2f} "
                         f"{'OK' if pose.is_reliable else 'LOW'} "
                         f"(smoothed {self._smoothed_confidence:.2f})   "
                         f"spread {pose.spread_mm:5.0f}mm   match {pose.match_error_mm:5.0f}mm")
            lines.append(f"heading from {self._heading_source} "
                         f"(wall quality {self._orientation_quality:.2f})")
            beams = 0 if self._last_bearings is None else len(self._last_bearings)
            fov = "360" if self.fov_deg is None else f"{self.fov_deg[0]:.0f}..{self.fov_deg[1]:.0f}"
            lines.append(f"{self.particle_count} particles   {beams} beams   "
                         f"fov {fov}deg   {self._update_ms:.1f}ms/update   #{self._updates}")
            lines.append(f"blocks: {self.blocks.summary()}")
        if truth is not None and pose is not None:
            error = math.hypot(truth[0] - pose.x, truth[1] - pose.y)
            lines.append(f"TRUTH ({truth[0] / 10:+.1f}, {truth[1] / 10:+.1f})cm "
                         f"@ {truth[2]:.1f}deg   error {error / 10:.1f}cm / "
                         f"{abs(angle_difference(truth[2], pose.heading)):.1f}deg")
        if truth_blocks:
            found, error = self.block_accuracy(truth_blocks)
            lines.append(f"BLOCKS {found}/{len(truth_blocks)} mapped"
                         + (f"   mean error {error / 10:.1f}cm" if found else ""))

        for i, line in enumerate(lines):
            ImageDrawingUtils.draw_label(canvas, line, (12, 22 + i * 19),
                                         (235, 235, 235), scale=0.45)

    def debug_text(self, truth=None):
        """The same readout as render_debug, for running over plain SSH."""
        pose = self.get_pose(max_age_s=None)
        if pose is None:
            return "[nav] not started"
        lines = [f"pose      {pose}",
                 f"smoothed  {self._smoothed_confidence:.2f} "
                 f"({'searching' if self._smoothed_confidence < RECOVERY_CONFIDENCE else 'locked'})",
                 f"spread    {pose.spread_mm:.0f}mm",
                 f"match     {pose.match_error_mm:.0f}mm",
                 f"heading   {self._heading_source} "
                 f"(wall quality {self._orientation_quality:.2f})",
                 f"update    {self._update_ms:.1f}ms  #{self._updates}",
                 self.blocks.debug_text()]
        if truth is not None:
            error = math.hypot(truth[0] - pose.x, truth[1] - pose.y)
            lines.append(f"truth     ({truth[0] / 10:+.1f}, {truth[1] / 10:+.1f})cm "
                         f"@ {truth[2]:.1f}deg  error {error / 10:.1f}cm")
        return "\n".join(lines)

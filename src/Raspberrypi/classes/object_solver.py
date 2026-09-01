import math
import threading
from dataclasses import dataclass

import cv2
import numpy as np

from utils.enums import Color
from utils.image_color_utils import ImageColorUtils
from utils.image_drawing_utils import ImageDrawingUtils
from utils.image_transform_utils import ImageTransformUtils

# ============================================================================
# Camera / lens configuration
# ============================================================================
# IMPORTANT: this is NOT the lens' rated 160 FOV. Picamera2's chosen sensor
# mode + the (CAMERA_PIC_WIDTH x CAMERA_PIC_HEIGHT) main-stream resize crop
# the sensor before any of this code sees a frame, so the *delivered* image
# covers a smaller angle than the lens spec sheet. Measure this directly
# against the actual frame (point the camera at two references a known angle
# apart, e.g. floor tiles or a printed protractor, and count pixels between
# them) rather than trusting the lens datasheet number.
CAPTURED_HORIZONTAL_FOV_DEG = 100.0

# Full, UNCROPPED main-stream width that CAPTURED_HORIZONTAL_FOV_DEG was
# measured against (see camera_manager.py's configure_camera/transform_image).
CAPTURED_FRAME_WIDTH_PX = ImageTransformUtils.CAMERA_PIC_WIDTH

# Degrees per pixel, assumed the SAME on both axes (square pixels + an
# equidistant/fisheye-style lens map angle linearly to pixel distance from the
# optical center regardless of the sensor's rectangular aspect ratio). Using
# one figure for both axes is what keeps the 5cm height measurement correct on
# a frame that isn't square.
DEG_PER_PX = CAPTURED_HORIZONTAL_FOV_DEG / CAPTURED_FRAME_WIDTH_PX

# Real-world HEIGHT of the boxes. The WRO traffic signs are 5cm x 5cm on the
# floor and 10cm tall - this is the 10, not the 5. Distance scales linearly
# with it, so the 5.0 that used to be here reported every pillar at half its
# true range.
BOX_HEIGHT_CM = 10.0
MIN_CONTOUR_AREA_PX = 160        # ignore specks smaller than this (noise)
MAX_DISTANCE_CM = 200.0         # ignore solves further than this (noise / grazing angle)
MIN_ANGULAR_HEIGHT_DEG = 0.05   # avoid a division by ~0 on a 1px-tall contour

TARGET_COLORS = (Color.GREEN, Color.RED)

# The parking bay's walls. Found separately from the pillars, and deliberately:
# they must never reach the block map (a bay wall folded in as a pillar is
# something the lap would swerve around), and they do not pass the tests the
# pillars are found by - see detect_parking.
PARKING_COLOR = Color.PINK
# Lower than the pillars' threshold: a bay wall is a low bar seen from a
# couple of metres away, which is a much smaller blob than a 10cm pillar at
# the same range, and it only has to be caught once while it is still ahead.
MIN_PARKING_AREA_PX = 150

# ============================================================================
# Top-down debug visualizer configuration
# ============================================================================
TOP_DOWN_CANVAS_PX = 500
TOP_DOWN_CM_PER_PX = 1.0        # canvas covers +/-250cm around the robot
TOP_DOWN_RING_STEP_CM = 50

COLOR_BGR = {
    Color.GREEN: (0, 255, 0),
    Color.RED: (0, 0, 255),
}
PARKING_BGR = (255, 0, 255)      # the bay walls, in the debug overlay


@dataclass
class ParkingMark:
    """
    A pink bay wall as the camera sees it: which way, and how big.

    NO DISTANCE, on purpose. Every range in DetectedObject is solved from the
    apparent height of a 10cm pillar; a bay wall is a different object of a
    different size, so that number would be confidently wrong. The bearing
    comes from the pixel centre and is independent of what size the thing is,
    so that is what this carries - enough to say "the bay is over there", which
    is all the camera is asked for.
    """
    bearing_deg: float      # + right of the camera axis, - left
    area_px: float
    width_px: float
    pixel_center: tuple
    box_points: np.ndarray


@dataclass
class DetectedObject:
    color: Color
    distance_cm: float   # straight-line distance from the camera to the box
    bearing_deg: float   # angle off the camera's forward axis, +right / -left
    forward_cm: float     # distance projected onto the robot's forward axis
    lateral_cm: float     # distance projected onto the robot's right axis
    pixel_center: tuple
    box_points: np.ndarray  # 4 corners of the detected box, for drawing


class ObjectSolver:
    """
    Detects GREEN and RED boxes in a frame and solves their position relative
    to the camera, using the boxes' known real-world height (BOX_HEIGHT_CM)
    and the lens' angular field of view.

    Coordinate convention (robot/camera frame, centimeters):
        forward_cm : + straight ahead of the camera
        lateral_cm : + to the camera's right, - to the left

    This assumes a level camera. It does not perform fisheye/lens
    undistortion - pixel offsets are mapped to angles linearly across the
    sensor (equidistant projection) using a single deg-per-pixel figure
    (DEG_PER_PX) shared by both axes, which is a reasonable approximation for
    wide-angle lenses without a full calibration, but will be least accurate
    right at the edges of the frame.

    Distance comes from the box's ANGULAR SIZE, which is independent of where
    the box sits in the frame - so camera_manager.transform_image() cropping
    away the top rows does not affect it. Bearing is measured from a single
    pixel column, though, so it does depend on the frame's horizontal origin;
    horizontal_crop_offset_px exists to correct that if the capture ever gets
    cropped horizontally (it currently isn't - PIC_WIDTH == CAMERA_PIC_WIDTH).
    """

    def __init__(self, debug=False, image_width=None, image_height=None,
                 horizontal_crop_offset_px=0.0):
        self.debug = debug
        self.image_width = image_width or ImageTransformUtils.PIC_WIDTH
        self.image_height = image_height or ImageTransformUtils.PIC_HEIGHT
        self.horizontal_crop_offset_px = horizontal_crop_offset_px
        self._last_parking = []

        # Debug canvases drawn by detect() and waiting to be put on screen.
        # detect() runs on VisionManager's thread (classes/vision_manager.py)
        # and HighGUI is main-thread only: an imshow() from a worker thread
        # does not raise on OpenCV 5's Qt backend, it prints
        #     QMetaObject::invokeMethod: No such method GuiReceiver::showImage
        # once per frame and drops the picture. So detect() only renders, and
        # show_debug() - called from the thread that owns the windows - is
        # what displays.
        self._debug_frames = {}
        self._debug_lock = threading.Lock()
        self._open_windows = set()

    def set_debug(self, enabled):
        self.debug = enabled

    def detect(self, hsv_image, display_image=None):
        """
        Finds GREEN/RED boxes in hsv_image and solves their position.

        I/O:
            hsv_image: HSV frame to search for boxes in
            display_image: optional BGR frame to draw debug overlays on
                            (only used when self.debug is True)
            return: list of DetectedObject, one per box found
        """
        objects = []
        for color in TARGET_COLORS:
            mask = ImageColorUtils.calculate_color_mask(hsv_image, color)
            objects.extend(self._find_objects_in_mask(mask, color))

        if self.debug:
            self._render_debug(objects, display_image)

        return objects

    def detect_parking(self, hsv_image):
        """
        Finds the pink parking-bay walls, biggest first.

        Not part of detect(). A bay wall is a long low bar - wider than it is
        tall - which is exactly the shape _find_objects_in_mask throws away as
        noise, since the pillars it looks for stand upright. And the result
        must stay out of nav.observe_blocks: the lap dodges what it finds
        there, and it should not dodge the thing it is trying to park in.

        I/O:
            hsv_image: HSV frame to search
            return: list of ParkingMark, largest area first
        """
        mask = ImageColorUtils.calculate_color_mask(hsv_image, PARKING_COLOR)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        marks = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_PARKING_AREA_PX:
                continue
            rect = cv2.minAreaRect(contour)
            (center_x, center_y), (width, height), _ = rect
            marks.append(ParkingMark(
                bearing_deg=round(self._horizontal_pixel_to_angle(center_x), 1),
                area_px=float(area),
                width_px=float(max(width, height)),
                pixel_center=(int(center_x), int(center_y)),
                box_points=np.intp(cv2.boxPoints(rect))))
        marks.sort(key=lambda mark: -mark.area_px)
        self._last_parking = marks      # for the debug overlay
        return marks

    def _find_objects_in_mask(self, mask, color):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects = []
        for cnt in contours:
            if cv2.contourArea(cnt) < MIN_CONTOUR_AREA_PX:
                continue
            _, _, box_w, box_h = cv2.boundingRect(cnt)
            if box_w > box_h:
                # Blocks stand upright, so a wider-than-tall bounding box is noise.
                continue
            obj = self._solve_position(cv2.minAreaRect(cnt), color)
            if obj is not None:
                objects.append(obj)
        return objects

    def _solve_position(self, rect, color):
        box_points = cv2.boxPoints(rect)
        center_x, center_y = rect[0]

        # The boxes are twice as tall as they are wide, so the LONGEST side of
        # the rotated rect is the height. Measuring the rect's own side (rather
        # than the vertical extent of its axis-aligned bounding box) keeps the
        # reading correct when the box appears tilted in frame - a tilted
        # upright box has a taller bounding box than it does an actual height,
        # which would otherwise read as closer than it really is.
        height_px = max(rect[1])
        angular_height_deg = height_px * DEG_PER_PX
        if angular_height_deg < MIN_ANGULAR_HEIGHT_DEG:
            return None

        distance_cm = (BOX_HEIGHT_CM / 2.0) / math.tan(math.radians(angular_height_deg / 2.0))
        if not (0 < distance_cm <= MAX_DISTANCE_CM):
            return None

        bearing_deg = self._horizontal_pixel_to_angle(center_x)
        forward_cm = distance_cm * math.cos(math.radians(bearing_deg))
        lateral_cm = distance_cm * math.sin(math.radians(bearing_deg))

        return DetectedObject(
            color=color,
            distance_cm=round(distance_cm, 1),
            bearing_deg=round(bearing_deg, 1),
            forward_cm=round(forward_cm, 1),
            lateral_cm=round(lateral_cm, 1),
            pixel_center=(int(center_x), int(center_y)),
            box_points=np.intp(box_points),
        )

    def _horizontal_pixel_to_angle(self, x):
        full_frame_x = x + self.horizontal_crop_offset_px
        return (full_frame_x - CAPTURED_FRAME_WIDTH_PX / 2.0) * DEG_PER_PX

    # ========================================================================
    # DEBUG VISUALIZATION
    # ========================================================================

    def _render_debug(self, objects, display_image):
        """Draws both debug views and parks them for the next show_debug()."""
        frames = {"Object Solver - Top Down": self._render_top_down(objects)}
        if display_image is not None:
            frames["Object Solver - Camera"] = self._render_camera_overlay(
                objects, display_image)
        with self._debug_lock:
            self._debug_frames = frames

    def show_debug(self):
        """
        Puts the last detect()'s debug views on screen.

        MAIN THREAD ONLY, and it needs a cv2.waitKey() after it to paint -
        same contract as NavigationManager.show_debug(). The round's debug
        loop provides both (tasks/path_task.py).

        I/O:
            return: True if there was a new frame to show
        """
        with self._debug_lock:
            frames, self._debug_frames = self._debug_frames, {}
        for window_name, canvas in frames.items():
            cv2.imshow(window_name, canvas)
            self._open_windows.add(window_name)
        return bool(frames)

    def close_debug(self):
        """Closes whatever show_debug() opened. Main thread, same as show."""
        for window_name in self._open_windows:
            cv2.destroyWindow(window_name)
        self._open_windows.clear()

    def _render_camera_overlay(self, objects, display_image):
        overlay = display_image.copy()
        for obj in objects:
            bgr = COLOR_BGR[obj.color]
            cv2.drawContours(overlay, [obj.box_points], 0, bgr, 2)
            label = f"{obj.distance_cm}cm {obj.bearing_deg}deg"
            ImageDrawingUtils.draw_label(overlay, label,
                                         (obj.pixel_center[0] - 45, obj.pixel_center[1] - 10), bgr)
        # The bay walls, when the park has asked for them. Drawn from the last
        # detect_parking rather than found again here: the point of showing
        # them is to see what the PARK is actually being given, not what a
        # second pass on a different frame would have found.
        for mark in self._last_parking:
            cv2.drawContours(overlay, [mark.box_points], 0, PARKING_BGR, 2)
            ImageDrawingUtils.draw_label(
                overlay, f"bay {mark.bearing_deg:+.0f}deg {mark.area_px:.0f}px",
                (mark.pixel_center[0] - 45, mark.pixel_center[1] - 10), PARKING_BGR)
        return overlay

    def _render_top_down(self, objects):
        size = TOP_DOWN_CANVAS_PX
        scale = TOP_DOWN_CM_PER_PX
        canvas = np.zeros((size, size, 3), dtype=np.uint8)
        origin = (size // 2, size - 30)  # robot near the bottom, forward = up

        self._draw_fov_cone(canvas, origin)
        self._draw_range_rings(canvas, origin, scale)

        cv2.circle(canvas, origin, 6, (255, 255, 255), -1)
        ImageDrawingUtils.draw_label(canvas, "ROBOT", (origin[0] - 30, origin[1] + 22), (255, 255, 255))

        for obj in objects:
            px = int(origin[0] + obj.lateral_cm / scale)
            py = int(origin[1] - obj.forward_cm / scale)
            if 0 <= px < size and 0 <= py < size:
                bgr = COLOR_BGR[obj.color]
                cv2.circle(canvas, (px, py), 6, bgr, -1)
                ImageDrawingUtils.draw_label(canvas, f"{obj.distance_cm}cm", (px + 10, py + 5), bgr)

        return canvas

    def _draw_fov_cone(self, canvas, origin):
        size = canvas.shape[0]
        working_horizontal_fov_deg = self.image_width * DEG_PER_PX
        half_fov_rad = math.radians(working_horizontal_fov_deg / 2.0)
        for sign in (-1, 1):
            end = (
                int(origin[0] + sign * math.sin(half_fov_rad) * size),
                int(origin[1] - math.cos(half_fov_rad) * size),
            )
            cv2.line(canvas, origin, end, (60, 60, 60), 1)

    @staticmethod
    def _draw_range_rings(canvas, origin, scale):
        size = canvas.shape[0]
        max_cm = int(size * scale)
        for r_cm in range(TOP_DOWN_RING_STEP_CM, max_cm, TOP_DOWN_RING_STEP_CM):
            r_px = int(r_cm / scale)
            cv2.circle(canvas, origin, r_px, (60, 60, 60), 1)
            # label on the up-left diagonal, not straight up: the vertical center
            # axis is where objects at ~0 bearing plot, and the two collide there
            offset = int(r_px * 0.7071)
            x, y = origin[0] - offset + 4, origin[1] - offset - 4
            if x < 4 or y < 12:   # ring's label would run off the canvas edge
                continue
            ImageDrawingUtils.draw_label(canvas, f"{r_cm}cm", (x, y), (150, 150, 150), scale=0.45)

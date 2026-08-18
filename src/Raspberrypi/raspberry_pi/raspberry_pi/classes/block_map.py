"""
The red and green pillars, pinned to the field instead of to the camera.

ObjectSolver answers "there is a red box 80cm ahead and 12 degrees right of the
camera". That is the right answer for dodging something in the next second, and
the wrong answer for everything else: the moment the pillar leaves the 40-degree
camera cone it stops existing, so the robot cannot plan a line through pillars
it has already driven past, and it re-discovers the same pillar four times a lap
as if it were four different ones.

BlockMap converts each detection into field coordinates using the pose from
NavigationManager, then merges detections that land in the same place into one
tracked block. What comes out is a map of the course that persists after the
camera looks away.

    nav.observe_blocks(object_solver.detect(camera.hsv_image))
    for block in nav.blocks.confirmed():
        print(block.color, block.x, block.y)

Blocks are 50mm square and 100mm tall. Orientation is not tracked - a square
looks the same from every side, and the camera cannot measure the yaw of one
anyway.

Everything here is in millimeters and the field frame from field_map.py, even
though ObjectSolver talks centimeters - the conversion happens once, on the way
in, so nothing downstream has to remember which unit it is holding.
"""
import math
import time
from dataclasses import dataclass, field

import cv2

from classes.field_map import FieldMap
from classes.object_solver import CAPTURED_HORIZONTAL_FOV_DEG
from utils.angle_utils import angle_difference, normalize_angle
from utils.enums import Color
from utils.image_drawing_utils import ImageDrawingUtils

# ============================================================================
# Block geometry (WRO Future Engineers traffic signs)
# ============================================================================
BLOCK_SIZE_MM = 50.0             # 50mm x 50mm footprint
BLOCK_HEIGHT_MM = 100.0          # 100mm tall - what ObjectSolver ranges off

# ObjectSolver measures the FRONT FACE of the block, since that is the surface
# whose height it sees. The block's center is half a block deeper.
FACE_TO_CENTER_MM = BLOCK_SIZE_MM / 2.0

# ============================================================================
# Mapping
# ============================================================================
# Two detections closer together than this are the same block. Comfortably
# bigger than the block itself, because the range estimate is what is uncertain,
# and comfortably smaller than the gap the rules leave between two pillars.
ASSOCIATION_RADIUS_MM = 180.0

# Range beyond which a detection is watched but not mapped. Distance comes from
# angular height, so its error grows with the square of range - at 2m a 5cm box
# is a handful of pixels tall and one pixel of contour noise moves it 20cm.
MAX_MAPPING_RANGE_MM = 1500.0

# A pose this uncertain would smear the block across the field, so detections
# taken during it are dropped rather than mapped into the wrong place.
MIN_POSE_CONFIDENCE = 0.35

# --- Confirmation / pruning model -------------------------------------
# A block goes through three states: TENTATIVE (< HITS_TO_CONFIRM sightings,
# not steered on), CONFIRMED (steered on), and DROPPED. Two independent
# things can drop a block, and both matter:
#
#   MAX_MISSES   ticks up only when the block SHOULD currently be in frame
#                (see _should_be_visible) and is not - a false positive from
#                one bad frame disappears instead of sitting on the map
#                forever.
#   MAX_STALE_S  a real-time backstop that does not care about visibility at
#                all. MAX_MISSES alone has a gap: a block mapped to a wrong
#                position that rarely ends up back in frame (behind you most
#                of the lap, hidden by the centre block) can dodge that check
#                indefinitely and steer the line off a phantom for most of a
#                lap. This forces every block to be RE-confirmed at least
#                this often or be dropped, no exceptions.
HITS_TO_CONFIRM = 2              # sightings before a block is believed - lower
                                  # than it looks: at CAMERA_EVERY_N_TICKS=5,
                                  # every missed/occluded frame along the way
                                  # burns into the approach_mm ramp before the
                                  # robot is even allowed to start dodging
MAX_MISSES = 6                   # unseen-but-should-be-visible frames before dropping
MAX_STALE_S = 6.0                # unconfirmed-regardless-of-visibility timeout
POSITION_SMOOTHING_FLOOR = 0.15  # a settled block can still be nudged this hard

# Blocks live on the mat, so anything solved into a wall is a bad detection.
BLOCK_EDGE_MARGIN_MM = 20.0

# ============================================================================
# Camera
# ============================================================================
# Taken from ObjectSolver rather than repeated here: it is a measured property
# of the lens + sensor mode, it has already been re-measured once, and a stale
# copy of it would silently break the "should I have seen that block?" test.
DEFAULT_CAMERA_FOV_DEG = CAPTURED_HORIZONTAL_FOV_DEG

# The lens sits this far straight back from the lidar on the same mast. Both
# sensors are described relative to the robot's center, but only ONE of the two
# distances is a fixed property of the build - so the camera's offset is
# derived from the lidar's rather than typed in twice and left to drift apart.
CAMERA_BEHIND_LIDAR_MM = 210.0
# Blocks at the very edge of the frame are half-cut and range badly, so the
# "should I have seen it?" test uses a slightly narrower cone than the real one.
VISIBILITY_FOV_MARGIN_DEG = 4.0

# ============================================================================
# Drawing
# ============================================================================
COLOR_BGR = {
    Color.GREEN: (70, 120, 60),
    Color.RED: (140, 30, 30),
}
UNCONFIRMED_BGR = (110, 110, 110)


def camera_offset_behind_lidar(lidar_offset_mm=(0.0, 0.0),
                               behind_mm=CAMERA_BEHIND_LIDAR_MM):
    """
    Where the lens is, given where the lidar is: `behind_mm` straight back
    along the robot's forward axis, same lateral position.

    I/O:
        lidar_offset_mm: (forward, right) of the lidar from the robot's center
        return: (forward, right) of the lens, in the same frame
    """
    forward, right = lidar_offset_mm
    return forward - behind_mm, right


@dataclass
class Block:
    """One tracked pillar, in field millimeters."""
    color: Color
    x: float
    y: float
    hits: int = 1
    misses: int = 0
    last_seen: float = field(default_factory=time.monotonic)

    @property
    def is_confirmed(self):
        return self.hits >= HITS_TO_CONFIRM

    @property
    def position(self):
        return self.x, self.y

    def __str__(self):
        state = "confirmed" if self.is_confirmed else f"{self.hits}/{HITS_TO_CONFIRM}"
        return f"{self.color.value:<5} ({self.x / 10:+6.1f}, {self.y / 10:+6.1f})cm [{state}]"


class BlockMap:
    """
    Tracks red/green pillars in field coordinates, fed by ObjectSolver
    detections plus the pose they were taken from.

    Association is nearest-neighbour by color inside ASSOCIATION_RADIUS_MM,
    which is all the structure this problem needs: the pillars do not move, the
    rules space them well apart, and a block that gets mis-associated will be
    voted back into place by the next few sightings anyway.

    Blocks that should be in shot but are not get a miss counted against them,
    so a false positive from a bad frame disappears instead of sitting on the
    map forever. "Should be in shot" accounts for the center block getting in
    the way, or a pillar behind it would be pruned every time the robot drove
    down the far side of the field.
    """

    def __init__(self, field_map=None, camera_offset_mm=None,
                 camera_yaw_deg=0.0, camera_fov_deg=DEFAULT_CAMERA_FOV_DEG,
                 max_range_mm=MAX_MAPPING_RANGE_MM, debug=False):
        """
        I/O:
            camera_offset_mm: (forward, right) of the lens from the robot's
                              center, in the robot's own frame. Defaults to
                              CAMERA_BEHIND_LIDAR_MM behind a centered lidar;
                              NavigationManager overrides this with the offset
                              measured from wherever the lidar actually is.
            camera_yaw_deg: how far the camera is turned off straight ahead
            camera_fov_deg: horizontal field of view, for the visibility test
        """
        self.map = field_map or FieldMap()
        if camera_offset_mm is None:
            camera_offset_mm = camera_offset_behind_lidar()
        self.camera_offset_mm = tuple(float(v) for v in camera_offset_mm)
        self.camera_yaw_deg = float(camera_yaw_deg)
        self.camera_fov_deg = float(camera_fov_deg)
        self.max_range_mm = float(max_range_mm)
        self.debug = debug

        self._blocks = []
        self.observations = 0
        self.rejected = 0

    # ========================================================================
    # READING
    # ========================================================================

    def all(self):
        """Every tracked block, confirmed or not."""
        return list(self._blocks)

    def confirmed(self):
        """Blocks seen enough times to steer on."""
        return [block for block in self._blocks if block.is_confirmed]

    def nearest(self, x, y, color=None, confirmed_only=True):
        """Closest tracked block to a point, or None."""
        candidates = self.confirmed() if confirmed_only else self._blocks
        if color is not None:
            candidates = [block for block in candidates if block.color == color]
        if not candidates:
            return None
        return min(candidates, key=lambda b: math.hypot(b.x - x, b.y - y))

    def clear(self):
        self._blocks = []

    def __len__(self):
        return len(self._blocks)

    # ========================================================================
    # MAPPING
    # ========================================================================

    def observe(self, pose, detections):
        """
        Folds one frame of ObjectSolver detections into the map.

        I/O:
            pose: the Pose the frame was taken from. Ignored entirely when it
                  is not reliable - a detection placed with a bad pose is
                  worse than no detection, because it lands somewhere real and
                  looks just as trustworthy as a good one.
            detections: list of DetectedObject from ObjectSolver.detect()
            return: list of the Blocks that were hit or created this frame
        """
        if pose is None or pose.confidence < MIN_POSE_CONFIDENCE:
            self.rejected += len(detections or ())
            return []

        camera_x, camera_y = self.camera_position(pose)
        seen = []
        for detection in detections or ():
            point = self._to_field(pose, camera_x, camera_y, detection)
            if point is None:
                self.rejected += 1
                continue
            seen.append(self._merge(detection.color, *point))

        self.observations += len(seen)
        self._count_misses(pose, camera_x, camera_y, seen)
        return seen

    def camera_position(self, pose):
        """Where the lens is in field coordinates, given the robot's pose."""
        forward, right = self.camera_offset_mm
        radians = math.radians(pose.heading)
        return (pose.x + forward * math.sin(radians) + right * math.cos(radians),
                pose.y + forward * math.cos(radians) - right * math.sin(radians))

    def _to_field(self, pose, camera_x, camera_y, detection):
        """
        One detection's field position, or None if it should not be mapped.

        ObjectSolver's bearing is relative to the camera's forward axis, so the
        field bearing is just the robot's heading plus the camera's mounting
        yaw plus that - all three are clockwise-positive, which is the whole
        reason the convention is shared.
        """
        distance = detection.distance_cm * 10.0 + FACE_TO_CENTER_MM
        if distance > self.max_range_mm:
            return None

        bearing = normalize_angle(pose.heading + self.camera_yaw_deg + detection.bearing_deg)
        radians = math.radians(bearing)
        x = camera_x + distance * math.sin(radians)
        y = camera_y + distance * math.cos(radians)

        # A pillar solved into a wall or inside the center block is a bad
        # detection (usually a colored reflection), not a pillar.
        if not self.map.contains(x, y, BLOCK_EDGE_MARGIN_MM):
            return None
        return x, y

    def _merge(self, color, x, y):
        """Updates the nearest matching block, or starts tracking a new one."""
        match = None
        best = ASSOCIATION_RADIUS_MM ** 2
        for block in self._blocks:
            if block.color != color:
                continue
            distance = (block.x - x) ** 2 + (block.y - y) ** 2
            if distance < best:
                best, match = distance, block

        if match is None:
            match = Block(color=color, x=x, y=y)
            self._blocks.append(match)
            if self.debug:
                print(f"[blocks] new {match}")
            return match

        # Running average that never fully freezes: early sightings move the
        # block a lot, later ones only trim it, but a block that is genuinely
        # in the wrong place can still be walked back.
        match.hits += 1
        alpha = max(POSITION_SMOOTHING_FLOOR, 1.0 / match.hits)
        match.x += alpha * (x - match.x)
        match.y += alpha * (y - match.y)
        match.misses = 0
        match.last_seen = time.monotonic()
        return match

    def _count_misses(self, pose, camera_x, camera_y, seen):
        """
        Penalizes blocks that were in shot and in range but did not turn up,
        and drops the ones that keep failing to.
        """
        now = time.monotonic()
        survivors = []
        for block in self._blocks:
            if block not in seen:
                if now - block.last_seen > MAX_STALE_S:
                    if self.debug:
                        print(f"[blocks] dropping {block} - stale "
                              f"({now - block.last_seen:.1f}s unconfirmed)")
                    continue
                if self._should_be_visible(pose, camera_x, camera_y, block):
                    block.misses += 1
                    if block.misses >= MAX_MISSES:
                        if self.debug:
                            print(f"[blocks] dropping {block} - not there")
                        continue
            survivors.append(block)
        self._blocks = survivors

    def _should_be_visible(self, pose, camera_x, camera_y, block):
        """True if this block ought to have shown up in the frame."""
        offset_x = block.x - camera_x
        offset_y = block.y - camera_y
        distance = math.hypot(offset_x, offset_y)
        if distance > self.max_range_mm or distance < 1.0:
            return False

        bearing = math.degrees(math.atan2(offset_x, offset_y))
        aim = pose.heading + self.camera_yaw_deg
        if abs(angle_difference(bearing, aim)) > (self.camera_fov_deg / 2.0
                                                  - VISIBILITY_FOV_MARGIN_DEG):
            return False

        # The center block hides everything behind it, and half the lap is
        # spent looking straight at it.
        sightline = float(self.map.raycast(camera_x, camera_y, math.radians(bearing)))
        return sightline >= distance - BLOCK_SIZE_MM

    # ========================================================================
    # DRAWING
    # ========================================================================

    def draw(self, canvas, to_px, scale):
        """
        Draws every tracked block onto a top-down canvas.

        I/O:
            to_px: field mm -> pixel callable, from the caller's renderer
            scale: pixels per mm, so the squares come out life-size
        """
        half = BLOCK_SIZE_MM / 2.0
        for block in self._blocks:
            color = COLOR_BGR.get(block.color, (200, 200, 200))
            corners = to_px(block.x - half, block.y + half), to_px(block.x + half, block.y - half)
            if block.is_confirmed:
                cv2.rectangle(canvas, corners[0], corners[1], color, -1)
                cv2.rectangle(canvas, corners[0], corners[1], (240, 240, 240), 1)
            else:
                # Hollow and grey until it has been seen enough times, so a
                # one-frame false positive is visibly different from a pillar.
                cv2.rectangle(canvas, corners[0], corners[1], UNCONFIRMED_BGR, 1)
                ImageDrawingUtils.draw_label(canvas, str(block.hits),
                                             (corners[1][0] + 3, corners[1][1]),
                                             UNCONFIRMED_BGR, scale=0.35)
        return canvas

    def draw_camera_cone(self, canvas, to_px, pose, color=(70, 70, 90)):
        """The camera's field of view, so it is obvious what the map can see."""
        camera_x, camera_y = self.camera_position(pose)
        origin = to_px(camera_x, camera_y)
        for side in (-1.0, 1.0):
            radians = math.radians(pose.heading + self.camera_yaw_deg
                                   + side * self.camera_fov_deg / 2.0)
            cv2.line(canvas, origin,
                     to_px(camera_x + self.max_range_mm * math.sin(radians),
                           camera_y + self.max_range_mm * math.cos(radians)), color, 1)
        return canvas

    def summary(self):
        counts = {color: 0 for color in COLOR_BGR}
        for block in self.confirmed():
            counts[block.color] = counts.get(block.color, 0) + 1
        pending = len(self._blocks) - len(self.confirmed())
        return (f"{counts.get(Color.GREEN, 0)} green  {counts.get(Color.RED, 0)} red  "
                f"({pending} pending, {self.rejected} rejected)")

    def debug_text(self):
        lines = [f"blocks    {self.summary()}"]
        lines.extend(f"  {block}" for block in sorted(self._blocks,
                                                      key=lambda b: (b.color.value, b.x)))
        return "\n".join(lines)

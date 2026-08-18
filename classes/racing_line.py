"""
The loop the robot drives: a rounded square in the corridor between the center
block and the outer wall.

    +-----------------------------+
    |    ,-------------------.    |   straights sit `wall_margin_mm` off the
    |   /                     \\  |   outer wall, corners are quarter circles
    |  |     +-----------+     |  |   of `corner_radius_mm`
    |  |     |  no-go    |     |  |
    |  |     +-----------+     |  |
    |   \\                     /   |
    |    `-------------------'    |
    +-----------------------------+

Two coordinate systems meet here, so both are named explicitly:

    field frame   x, y in mm from the middle of the mat (field_map.py)
    path frame    progress along the loop in mm, and `lateral` - signed
                  perpendicular offset, POSITIVE TO THE RIGHT of the direction
                  of travel

`progress` always increases as the robot drives, whichever way round it is
going, which is what makes lap counting a subtraction and lets the same code
run clockwise and counter-clockwise. Everything takes a `direction` of +1 or
-1; +1 is the direction the geometry is built in (counter-clockwise seen from
above, i.e. +X along the bottom edge).

The 90-degree symmetry note from navigation_manager.py matters here: this loop
is invariant under a 90-degree rotation, so even if the localizer has settled
into the wrong quadrant of a symmetric map, the path it drives in the real
world is the correct one. Being "lost" by exactly a quadrant is harmless.
"""
import math

import numpy as np

from classes.field_map import FieldMap
from utils.angle_utils import angle_difference, clamp, normalize_angle

# How much room the corner arcs leave between themselves and the center block.
# Must exceed blocks.robot_half_width_mm or the corner apex clips the block
# even with zero pillar dodge active - 200 gives 40mm of real body clearance
# over the current 160mm robot_half_width_mm.
CORNER_BLOCK_MARGIN_MM = 170.0
DEFAULT_RESOLUTION_MM = 20.0


class RacingLine:
    """
    A closed rounded-square path, sampled into a polyline for projection and
    kept analytic for lookahead.

    Usage:
        path = RacingLine(field_map)
        direction = path.direction_for(pose)          # +1 or -1
        progress, lateral = path.project(pose.x, pose.y, direction)
        target = path.point_at(progress + 400, direction)
    """

    def __init__(self, field_map=None, wall_margin_mm=None, corner_radius_mm=None,
                 resolution_mm=DEFAULT_RESOLUTION_MM):
        """
        I/O:
            wall_margin_mm: gap from the outer wall to the straights. None
                            centers the loop in the corridor, which is the
                            most forgiving line and the sane default.
            corner_radius_mm: None picks the largest radius that still clears
                              the center block by CORNER_BLOCK_MARGIN_MM.
        """
        self.map = field_map or FieldMap()

        if wall_margin_mm is None:
            self.half = (self.map.outer + self.map.inner) / 2.0
        else:
            self.half = self.map.outer - float(wall_margin_mm)

        # The arcs pull in to (half - radius), which has to stay outside the
        # block. Capped at 70% of the half-size too, or the "square" collapses
        # into a circle and the straights vanish.
        largest = min(self.half - self.map.inner - CORNER_BLOCK_MARGIN_MM, self.half * 0.7)
        self.corner_radius = float(corner_radius_mm) if corner_radius_mm else largest
        self.corner_radius = clamp(self.corner_radius, 50.0, self.half * 0.95)

        self._segments = self._build_segments()
        self.length = sum(segment["length"] for segment in self._segments)

        self.resolution_mm = float(resolution_mm)
        self.points, self.headings, self.arc = self._sample()

    # ========================================================================
    # GEOMETRY
    # ========================================================================

    def _build_segments(self):
        """
        The loop as four straights and four quarter-circles, in the +1
        direction: +X along the bottom, up the right side, -X along the top,
        down the left. Headings are degrees clockwise from +Y throughout.

        The first straight is split in half so the loop starts at the middle
        of the bottom edge rather than at a corner - it makes no difference to
        the driving, but it makes the numbers easier to read while debugging.
        """
        half, radius = self.half, self.corner_radius
        straight = 2.0 * (half - radius)
        arc = math.pi / 2.0 * radius
        inner = half - radius

        return [
            {"line": ((0.0, -half), 90.0), "length": straight / 2.0},
            {"arc": ((inner, -inner), radius, -90.0), "length": arc},
            {"line": ((half, -inner), 0.0), "length": straight},
            {"arc": ((inner, inner), radius, 0.0), "length": arc},
            {"line": ((inner, half), 270.0), "length": straight},
            {"arc": ((-inner, inner), radius, 90.0), "length": arc},
            {"line": ((-half, inner), 180.0), "length": straight},
            {"arc": ((-inner, -inner), radius, 180.0), "length": arc},
            {"line": ((-inner, -half), 90.0), "length": straight / 2.0},
        ]

    def _pose_at_arc(self, arc_length):
        """
        (x, y, heading) at `arc_length` along the loop in the +1 direction.
        Analytic, so lookahead never quantizes to the sample spacing.
        """
        remaining = arc_length % self.length
        for segment in self._segments:
            if remaining > segment["length"]:
                remaining -= segment["length"]
                continue
            if "line" in segment:
                (start_x, start_y), heading = segment["line"]
                radians = math.radians(heading)
                return (start_x + remaining * math.sin(radians),
                        start_y + remaining * math.cos(radians), heading)
            (center_x, center_y), radius, start_angle = segment["arc"]
            angle = math.radians(start_angle + math.degrees(remaining / radius))
            # A point at math-angle `a` on a circle travelled counter-clockwise
            # has tangent heading -a in the clockwise-from-+Y convention.
            return (center_x + radius * math.cos(angle),
                    center_y + radius * math.sin(angle),
                    normalize_angle(-math.degrees(angle)))
        raise AssertionError("arc length past the end of the loop")

    def _sample(self):
        count = max(16, int(round(self.length / self.resolution_mm)))
        arc = np.linspace(0.0, self.length, count, endpoint=False)
        poses = [self._pose_at_arc(value) for value in arc]
        points = np.array([(pose[0], pose[1]) for pose in poses])
        headings = np.array([pose[2] for pose in poses])
        return points, headings, arc

    # ========================================================================
    # PATH FRAME
    # ========================================================================

    def _to_arc(self, progress, direction):
        """Travel-frame progress -> the underlying +1-direction arc length."""
        return (progress if direction > 0 else self.length - progress) % self.length

    def pose_at(self, progress, direction=1):
        """
        (x, y, heading) at a travel-frame progress. The heading is the way the
        robot faces going that way round, so it flips with `direction`.
        """
        x, y, heading = self._pose_at_arc(self._to_arc(progress, direction))
        return x, y, heading if direction > 0 else normalize_angle(heading + 180.0)

    def point_at(self, progress, direction=1, lateral_mm=0.0):
        """
        The point `lateral_mm` to the right of the path at that progress.
        This is what pure pursuit chases, and the lateral offset is how the
        final round steps around a pillar without needing a second path.
        """
        x, y, heading = self.pose_at(progress, direction)
        if not lateral_mm:
            return x, y
        # right-of-travel unit vector for a clockwise-from-+Y heading
        radians = math.radians(heading)
        return x + lateral_mm * math.cos(radians), y - lateral_mm * math.sin(radians)

    def project(self, x, y, direction=1):
        """
        Where a field point sits on the path.

        I/O:
            return: (progress_mm, lateral_mm) in the travel frame. `lateral` is
                    positive to the right of travel, so on a loop driven the +1
                    way a point outside the loop (toward the wall) is positive.

        The nearest SAMPLE gets the search down to one array op; the along-track
        component of the remaining offset then refines progress off the sample
        grid, so a 20mm polyline still gives a smooth, continuous progress.
        """
        offsets = self.points - np.array([x, y])
        index = int(np.argmin(np.einsum("ij,ij->i", offsets, offsets)))

        heading = math.radians(self.headings[index])
        forward = (math.sin(heading), math.cos(heading))
        right = (math.cos(heading), -math.sin(heading))
        offset_x = x - self.points[index][0]
        offset_y = y - self.points[index][1]

        along = offset_x * forward[0] + offset_y * forward[1]
        lateral = offset_x * right[0] + offset_y * right[1]
        arc = (self.arc[index] + along) % self.length

        if direction > 0:
            return arc, lateral
        return (self.length - arc) % self.length, -lateral

    def gap(self, from_progress, to_progress):
        """
        Signed distance from one progress to another, wrapped into
        +/- half a lap. Positive means `to` is ahead. This is the primitive
        for both lap counting and "is that pillar in front of me?".
        """
        return (to_progress - from_progress + self.length / 2.0) % self.length - self.length / 2.0

    # ========================================================================
    # DIRECTION
    # ========================================================================

    def direction_for(self, pose):
        """
        Which way round the loop the robot is already pointing: +1 or -1.

        The robot may be set down anywhere in any orientation, so the round
        starts by picking whichever direction needs the smaller turn. That
        caps the initial heading error at 90 degrees, which pure pursuit can
        drive out inside the corridor.
        """
        arc, _ = self.project(pose.x, pose.y, direction=1)
        _, _, path_heading = self.pose_at(arc, direction=1)
        return 1 if abs(angle_difference(path_heading, pose.heading)) <= 90.0 else -1

    @staticmethod
    def direction_name(direction):
        """Which way that is when you are standing over the mat."""
        return "counter-clockwise" if direction > 0 else "clockwise"

    # ========================================================================
    # LIMITS
    # ========================================================================

    def lateral_limit(self, clearance_mm):
        """
        How far the line may be shifted sideways before the robot runs out of
        corridor. The straights are the binding case: they sit `half` from the
        center, with the wall at `outer` and the block at `inner`.
        """
        toward_wall = self.map.outer - self.half - clearance_mm
        toward_block = self.half - self.map.inner - clearance_mm
        return max(0.0, min(toward_wall, toward_block))

    def curvature_at(self, progress, direction=1):
        """
        1/radius at that progress: 0 on the straights, 1/corner_radius in a
        corner. Used to back off the throttle before a corner rather than
        during it.
        """
        arc = self._to_arc(progress, direction)
        remaining = arc
        for segment in self._segments:
            if remaining > segment["length"]:
                remaining -= segment["length"]
                continue
            return 0.0 if "line" in segment else 1.0 / self.corner_radius
        return 0.0

    def max_curvature_between(self, progress, ahead_mm, direction=1, samples=6):
        """The sharpest bend in the next `ahead_mm` - what the throttle reads."""
        steps = np.linspace(0.0, ahead_mm, samples)
        return max(self.curvature_at(progress + step, direction) for step in steps)

    # ========================================================================
    # DRAWING
    # ========================================================================

    def draw(self, canvas, to_px, color=(70, 110, 70), thickness=1):
        """Draws the loop onto a top-down canvas (see NavigationManager)."""
        import cv2

        pixels = np.array([to_px(x, y) for x, y in self.points], dtype=np.int32)
        cv2.polylines(canvas, [pixels], isClosed=True, color=color, thickness=thickness)
        return canvas

    def __str__(self):
        return (f"RacingLine half={self.half:.0f}mm corner_r={self.corner_radius:.0f}mm "
                f"length={self.length:.0f}mm ({len(self.points)} samples)")

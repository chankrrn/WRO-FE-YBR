"""
The WRO Future Engineers field as geometry the localizer can raycast against.

    +Y (north)
     ^
     |   +-------------------------+  y = +1600
     |   |                         |
     |   |        +-------+        |  the 1000mm no-go block, y = +/-500
     |   |        |       |        |
     |   |        +-------+        |
     |   |                         |
     |   +-------------------------+  y = -1600
     +----------------------------------> +X (east)
        x = -1600                x = +1600

Everything is millimeters, the origin is the middle of the field, and the
drivable area is the 1100mm-wide ring between the outer walls and the block.

Angle convention (shared with LidarManager and CompassManager):
    a heading/bearing is degrees CLOCKWISE from +Y, so 0 = north, 90 = east.
    The unit vector for angle `a` is therefore (sin a, cos a), not the usual
    (cos a, sin a) - worth remembering when reading the raycaster.

Because both boxes are axis aligned, a ray cast is just two slab tests, which
vectorizes to "every particle against every beam" in one numpy expression.
That is the whole reason localization is cheap enough to run at lidar rate on
the Pi - see NavigationManager.
"""
import math

import numpy as np

# ============================================================================
# Field dimensions (WRO Future Engineers)
# ============================================================================
FIELD_SIZE_MM = 3000.0       # inner face to inner face of the outer walls
INNER_SIZE_MM = 1000.0       # the walled no-go block in the middle


class FieldMap:
    """Outer box + centered inner box, with a raycaster and an inside test."""

    def __init__(self, field_size_mm=FIELD_SIZE_MM, inner_size_mm=INNER_SIZE_MM):
        self.field_size_mm = float(field_size_mm)
        self.inner_size_mm = float(inner_size_mm)
        self.outer = self.field_size_mm / 2.0      # +/- this is the outer wall
        self.inner = self.inner_size_mm / 2.0      # +/- this is the block face

        # Extra axis-aligned boxes the lidar can see but the two fixed walls do
        # not explain - in practice the parking bay's two magenta walls, once
        # they have been found. See add_obstacle for why these are raycast-only.
        self.obstacles = []

    # ========================================================================
    # OBSTACLES
    # ========================================================================

    def add_obstacle(self, low, high):
        """
        Registers an axis-aligned box for the RAYCASTER only.

        Deliberately not part of contains(): that is the particle filter's
        "could the robot be here" test, and a particle inside the box is killed
        outright (see NavigationManager, log_weights -> -inf). The parking bay's
        walls are 100mm tall and the robot is supposed to end up BETWEEN them,
        so treating them as no-go would delete every particle in the bay at
        exactly the moment the pose matters most.

        What they ARE good for is prediction: the beams that hit them are
        otherwise unexplained, which inflates the match error and drags
        confidence down near the bay. Raycasting them makes those beams agree
        with the map instead.

        I/O:
            low, high: (x_min, y_min), (x_max, y_max) in mm
        """
        self.obstacles.append((tuple(float(v) for v in low),
                               tuple(float(v) for v in high)))
        return self

    def set_obstacles(self, rects):
        """Replaces the obstacle list with `rects`, a sequence of (low, high)."""
        self.obstacles = []
        for low, high in rects:
            self.add_obstacle(low, high)
        return self

    def clear_obstacles(self):
        self.obstacles = []
        return self

    def obstacle_rectangles(self):
        """The obstacles as (low, high) pairs, for rendering."""
        return tuple(self.obstacles)

    # ========================================================================
    # GEOMETRY QUERIES
    # ========================================================================

    @property
    def corridor_width_mm(self):
        """How wide the drivable ring is between the block and the wall."""
        return self.outer - self.inner

    @property
    def diagonal_mm(self):
        """Longest sightline in the field - the cap for any expected range."""
        return math.hypot(self.field_size_mm, self.field_size_mm)

    def contains(self, x, y, margin_mm=0.0):
        """
        True where (x, y) is on the drivable ring, i.e. inside the outer walls
        and outside the block, with `margin_mm` of clearance from both.

        I/O:
            x, y: scalars or numpy arrays of millimeters
            return: bool of the same (broadcast) shape
        """
        x = np.abs(np.asarray(x, dtype=float))
        y = np.abs(np.asarray(y, dtype=float))
        inside_field = (x <= self.outer - margin_mm) & (y <= self.outer - margin_mm)
        inside_block = (x < self.inner + margin_mm) & (y < self.inner + margin_mm)
        return inside_field & ~inside_block

    def raycast(self, x, y, angles_rad, max_range_mm=None, include_obstacles=True):
        """
        Distance from (x, y) to the first wall along each angle.

        Fully broadcast: pass x/y with shape (N, 1) and angles with shape
        (M,) to get the (N, M) range table for N particles by M beams.

        I/O:
            x, y: ray origins in mm
            angles_rad: bearings in radians, clockwise from +Y
            max_range_mm: clip the result (None = clip at the field diagonal)
            include_obstacles: fold in anything add_obstacle() registered.
                               False answers "what would a scan plane mounted
                               ABOVE the parking walls see", which is the one
                               hardware question this whole feature rests on.
            return: float array of millimeters, broadcast to the common shape
        """
        dx = np.sin(angles_rad)
        dy = np.cos(angles_rad)
        x, y, dx, dy = np.broadcast_arrays(np.asarray(x, dtype=float),
                                           np.asarray(y, dtype=float), dx, dy)

        # Outer walls: the ray starts inside, so the hit is where it EXITS.
        _, exit_x = self._slab(x, dx, -self.outer, self.outer)
        _, exit_y = self._slab(y, dy, -self.outer, self.outer)
        outer_hit = np.minimum(exit_x, exit_y)

        # Inner block: the ray starts outside, so the hit is where it ENTERS,
        # and only counts if it enters before it leaves and does so ahead of us.
        ranges = np.minimum(outer_hit, self._box_hit(x, y, dx, dy,
                                                     (-self.inner, -self.inner),
                                                     (self.inner, self.inner)))
        if include_obstacles:
            for low, high in self.obstacles:
                ranges = np.minimum(ranges, self._box_hit(x, y, dx, dy, low, high))
        cap = self.diagonal_mm if max_range_mm is None else max_range_mm
        return np.clip(ranges, 0.0, cap)

    @classmethod
    def _box_hit(cls, x, y, dx, dy, low, high):
        """
        Distance to where a ray STARTING OUTSIDE an axis-aligned box first
        enters it, or inf if it never does.

        Shared by the centre block and every registered obstacle - they are the
        same test, and the only reason the block was ever written out longhand
        is that it used to be the only box.
        """
        enter_x, leave_x = cls._slab(x, dx, low[0], high[0])
        enter_y, leave_y = cls._slab(y, dy, low[1], high[1])
        enter = np.maximum(enter_x, enter_y)
        leave = np.minimum(leave_x, leave_y)
        return np.where((enter <= leave) & (enter > 0), enter, np.inf)

    @staticmethod
    def _slab(p, d, low, high):
        """
        The interval of t for which p + t*d stays inside [low, high] on one
        axis. A ray parallel to the slab is either always in it or never in
        it, which is why the parallel case is patched in afterwards rather
        than left as the 0/0 the division produces.
        """
        with np.errstate(divide="ignore", invalid="ignore"):
            t_low = (low - p) / d
            t_high = (high - p) / d
        enter = np.minimum(t_low, t_high)
        leave = np.maximum(t_low, t_high)

        parallel = d == 0.0
        if np.any(parallel):
            between = (p >= low) & (p <= high)
            enter = np.where(parallel, np.where(between, -np.inf, np.inf), enter)
            leave = np.where(parallel, np.where(between, np.inf, -np.inf), leave)
        return enter, leave

    # ========================================================================
    # START POSITIONS
    # ========================================================================

    def start_zones(self):
        """
        The four cells the robot may legally be placed in.

        The mat is a 3x3 grid whose middle cell is the no-go block, so the
        grid lines fall on +/-`inner`. The robot never starts in a CORNER
        cell - only in the four edge-middle ones, which is what this returns.

        I/O:
            return: [(name, (x_min, y_min), (x_max, y_max)), ...] going
                    south, east, north, west
        """
        outer, inner = self.outer, self.inner
        return [
            ("south", (-inner, -outer), (inner, -inner)),
            ("east", (inner, -inner), (outer, inner)),
            ("north", (-inner, inner), (inner, outer)),
            ("west", (-outer, -inner), (-inner, inner)),
        ]

    @staticmethod
    def start_headings(zone_name):
        """
        The two orientations the robot can be placed in, in a given start
        cell: pointing either way along the track. Degrees clockwise from +Y.

        A south or north cell has the track running along X, so the robot
        faces east (90) or west (270); the east and west cells are the other
        way round.
        """
        return (90.0, 270.0) if zone_name in ("south", "north") else (0.0, 180.0)

    def start_poses(self):
        """Every legal (name, x, y, heading) - the centre of each cell, both ways."""
        poses = []
        for name, low, high in self.start_zones():
            center = ((low[0] + high[0]) / 2.0, (low[1] + high[1]) / 2.0)
            for heading in self.start_headings(name):
                poses.append((name, center[0], center[1], heading))
        return poses

    # ========================================================================
    # DRAWING
    # ========================================================================

    def rectangles(self):
        """
        The two boxes as ((x_min, y_min), (x_max, y_max)) pairs, outer first.
        Only used for rendering the debug view.
        """
        return (((-self.outer, -self.outer), (self.outer, self.outer)),
                ((-self.inner, -self.inner), (self.inner, self.inner)))

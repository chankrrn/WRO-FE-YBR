"""
Where things are on the robot, relative to THE POINT THE POSE DESCRIBES.

That point is the middle of the REAR (driving) AXLE. Everything that reasons
about the robot's motion is defined from there: the bicycle model turns about
the rear axle, pure pursuit draws its arc through it, the dead-reckoning
between lidar scans integrates it, and the planner's body sweep and the
parking geometry measure their overhangs from it. Making the sensors the
origin instead - which is what the localizer did when the lidar sat at
(0, 0) - put a point 15cm ahead of the axle into all of those, so every
turn dragged the "robot" sideways by the lidar's own swing.

The sensors sit on a mast at the front of the car, nearly over the steering
wheels, so they are described here as offsets from the axle and the pose is
transformed to them wherever a beam or a camera ray has to start. Offsets
are (forward, right) in millimetres in the robot's own frame.

MEASURE THESE with a ruler on the real car, from the rear axle's centreline.
"""
import numpy as np

# The lidar beam's origin, forward of the rear axle. 15cm: the mast stands
# almost on top of the front wheels and the wheelbase is 165mm.
LIDAR_AHEAD_MM = 150.0
DEFAULT_LIDAR_OFFSET_MM = (LIDAR_AHEAD_MM, 0.0)

# The lens sits this far straight back from the lidar on the same mast, so the
# camera's offset is derived from the lidar's rather than typed in twice and
# left to drift apart - move the mast and both sensors follow.
CAMERA_BEHIND_LIDAR_MM = 170.0


def to_field(x, y, heading_deg, offset_mm):
    """
    A point fixed to the robot, in field coordinates.

    Works on scalars and on numpy arrays alike (a particle cloud passes whole
    arrays through here every filter tick).

    I/O:
        x, y: the pose point (rear axle) in field mm
        heading_deg: degrees clockwise from +Y, so forward is (sin, cos) and
                     right is (cos, -sin)
        offset_mm: (forward, right) of the point from the axle
        return: (field_x, field_y)
    """
    forward, right = offset_mm
    radians = np.radians(heading_deg)
    sin, cos = np.sin(radians), np.cos(radians)
    return (x + forward * sin + right * cos,
            y + forward * cos - right * sin)


def camera_offset_behind_lidar(lidar_offset_mm=DEFAULT_LIDAR_OFFSET_MM,
                               behind_mm=CAMERA_BEHIND_LIDAR_MM):
    """
    Where the lens is, given where the lidar is: `behind_mm` straight back
    along the robot's forward axis, same lateral position.

    I/O:
        lidar_offset_mm: (forward, right) of the lidar from the rear axle
        return: (forward, right) of the lens, in the same frame
    """
    forward, right = lidar_offset_mm
    return forward - behind_mm, right

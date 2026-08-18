"""Angle bookkeeping shared by the compass, the navigator and the tasks."""


def normalize_angle(angle):
    """Wraps any angle into [0, 360)."""
    return angle % 360.0


def angle_difference(target, current):
    """
    Shortest signed turn from `current` to `target`, in (-180, 180].
    Positive means target is clockwise of current.
    """
    return (target - current + 180.0) % 360.0 - 180.0


def clamp(value, low, high):
    return max(low, min(high, value))

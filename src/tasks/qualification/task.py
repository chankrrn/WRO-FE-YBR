"""
Open round: three laps of the empty mat, from anywhere on it.
"""
from tasks.path_task import PathDrivingTask


class QualificationTask(PathDrivingTask):
    """
    Drive the racing line for `laps.goal` laps and stop.

    There is nothing to add to PathDrivingTask - the whole round IS the path
    following. The robot can be set down anywhere in any orientation; setup
    localizes, then picks whichever way round the loop needs the smaller turn,
    so the direction is decided by how the robot was placed rather than by
    hunting for a coloured line.

    Tunables live in config.toml beside this file.
    """

    name = "qualification"
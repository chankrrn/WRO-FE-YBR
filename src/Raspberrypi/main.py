"""
Dispatcher for the two competition rounds.

    uv run python main.py qualification
    uv run python main.py final --debug

Each round is also runnable directly, which is what the pit crew should use:

    uv run python -m tasks.qualification.run
    uv run python -m tasks.final.run

All the hardware now lives behind TaskContext (classes/task_context.py), which
owns one manager per subsystem:

    context.board       expansion board / ADC        classes/board_manager.py
    context.motor       drive + steering (Arduino)   classes/motor_manager.py
    context.compass     BNO055 heading               classes/compass_manager.py
    context.ultra       ultrasonic head + servo      classes/ultra_servo_manager.py
    context.lights      corner line sensors          classes/light_sensor_manager.py
    context.lidar       RPLidar C1 (opt-in)          classes/lidar_manager.py
    context.camera      Picamera2 (opt-in)           camera_manager.py
    context.navigator   wall-following (legacy)      classes/navigator.py
    context.nav         where we are on the field    classes/navigation_manager.py

Both rounds now drive the same way, from tasks/path_task.py: build a rounded
square in the corridor (classes/racing_line.py), work out which way round the
robot is pointing, and chase it with pure pursuit (classes/pure_pursuit.py).
The qualification round is exactly that; the final round overrides one method
to shift the line sideways past each pillar. context.navigator, the ultrasonic
head and the line sensors are no longer in that loop.

Speeds and geometry are per-round TOML, not code:

    tasks/qualification/config.toml
    tasks/final/config.toml

    uv run python main.py qualification --dry-run    check the plan, no hardware
    uv run python main.py qualification --speed 65   override for one run

--no-drive is the halfway house between --dry-run and a real run: every
sensor, the filter and the control loop all run for real and the steering
servo answers them tick by tick, but the drive motor is never given a speed,
so you push the robot round the mat by hand and watch where it wanted to go.

    uv run python main.py final --debug --no-drive

Test the whole round with no robot at all - real task, real filter, real
config, simulated chassis:

    uv run python test_driving.py --trials 24

context.nav is the odd one out: it is not a sensor, it is the lidar and the
compass matched against a map of the field (classes/field_map.py) to work out
where the robot actually is. It is started for every round and answers:

    pose = context.nav.get_pose()      # Pose(x, y, heading, confidence)
    if pose.is_reliable:               # x/y in mm from the middle of the field,
        steer_toward(pose.x, pose.y)   # heading in degrees clockwise from +Y

The pose is the middle of the REAR (driving) AXLE, not the lidar. The lidar
stands on a mast at the front, robot.lidar_ahead_mm (15cm) ahead of the axle,
and the filter casts its rays from there while the particles it localizes are
axle positions. That is the point the bicycle model, pure pursuit, the
odometry and the planner's body sweep are all written from; see
classes/robot_geometry.py.

Without the lidar (--lidar, or a round that requires it) every pose comes back
with confidence 0, so is_reliable is the only check a task needs. Pass
--start-pose X Y HEADING when you know where the robot is placed: it saves the
filter a second of searching and, if the compass is also dead, stops it picking
the wrong quadrant of a map that is 90-degree rotationally symmetric.

    uv run python main.py final --debug --lidar --start-pose 1050 -1400 90

context.nav also keeps a map of the red/green pillars (classes/block_map.py).
The final round feeds it every camera frame, which turns ObjectSolver's "a red
box 80cm ahead of the lens" into "a red block at this spot on the field" that
stays there after the camera looks away:

    for block in context.nav.blocks.confirmed():
        print(block.color, block.x, block.y)

The lens is assumed to sit 17cm straight back from the lidar
(robot_geometry.py's CAMERA_BEHIND_LIDAR_MM), so moving the mast with
robot.lidar_ahead_mm moves both sensors together. Override
context.nav.blocks.camera_offset_mm - (forward, right) from the rear axle -
only if the camera is mounted somewhere else entirely.

Run `uv run python test_navigation.py` to watch the whole thing localize
against a simulated field, with no hardware plugged in at all, or
`uv run python test_navigation.py --hardware --camera` to map blocks for real.
"""
import sys

from tasks.cli import run_task
from tasks.final.task import FinalTask
from tasks.qualification.task import QualificationTask

TASKS = {
    QualificationTask.name: QualificationTask,
    FinalTask.name: FinalTask,
}


def main(argv):
    if not argv or argv[0] in ("-h", "--help") or argv[0] not in TASKS:
        print(f"usage: python main.py {{{'|'.join(TASKS)}}} [options]\n"
              f"       python main.py <round> --help   for the per-round options")
        return 1 if argv and argv[0] not in ("-h", "--help") else 0

    # Everything after the round name is the round's own CLI (--laps, --debug...)
    return 0 if run_task(TASKS[argv[0]], argv[1:]) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

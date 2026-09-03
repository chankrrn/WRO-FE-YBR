"""
Parking-only entry point - the manoeuvre with no lap and no pillars.

    uv run python -m tasks.parking_test.run --wall right
    uv run python -m tasks.parking_test.run --wall left --debug
    uv run python -m tasks.parking_test.run --wall right --no-drive

The obstacle round is a separate script and is unchanged:

    uv run python -m tasks.final.run

Takes every flag tasks/cli.py offers, plus --wall, which this needs because
there is no lap here to work the direction out from.
"""
import sys
from pathlib import Path

from tasks.cli import OPENCV_THREADS, build_parser, load_config
from tasks.base_task import DEFAULT_MAX_RUNTIME_S
from tasks.parking_test.task import ParkingTestTask
from classes.veloz_parking import VelozParking
from classes.task_context import TaskContext


def main(argv=None):
    parser = build_parser("Run the parking manoeuvre on its own")
    parser.add_argument("--wall", choices=("left", "right"), default="right",
                        help="which side of the robot the outer wall (and so "
                             "the bay) is on (default: right)")
    args = parser.parse_args(argv)
    # ONE SET OF NUMBERS. There is deliberately no config.toml beside this
    # task: it reads the round's, so a value tuned on the bench here is the
    # same value the round drives with. --config still overrides.
    if not args.config:
        args.config = str(Path(__file__).resolve().parent.parent
                          / "final" / "config.toml")
    config = load_config(ParkingTestTask, args)

    # --dry-run must not open a serial port, spin a motor or start the lidar.
    # It prints the numbers the park WOULD run with and stops there.
    if args.dry_run:
        print(f"\nParking test - outer wall on the {args.wall}.")
        print(f"  config: {args.config}")
        park = VelozParking(
            wall_side=1.0 if args.wall == "right" else -1.0,
            search_drives=True,
            camera_confirms=bool(config.get("parking.camera_confirms", True)),
            align_stop_mm=float(config.get("parking.align_stop_mm", 300.0)),
            reverse_stop_mm=float(config.get("parking.reverse_stop_mm", 150.0)),
            min_reverse_mm=float(config.get("parking.min_reverse_mm", 80.0)),
            max_reverse_mm=float(config.get("parking.max_reverse_mm", 480.0)),
            straighten_extra_mm=float(
                config.get("parking.straighten_extra_mm", 120.0)),
            speed=int(config.get("parking.speed", 55)),
            reverse_speed=int(config.get("parking.reverse_speed", 60)),
            timeout_s=float(config.get("parking.timeout_s", 20.0)))
        print(f"  {park.summary()}")
        print(f"  reverse: budget from the bay beam less "
              f"{park.reverse_stop_mm:.0f}mm, clamped "
              f"{park.min_reverse_mm:.0f}-{park.max_reverse_mm:.0f}mm, "
              f"plus {park.straighten_extra_mm:.0f}mm to square up\n")
        return True

    max_runtime = args.max_runtime
    if max_runtime is None:
        max_runtime = None if args.no_drive else DEFAULT_MAX_RUNTIME_S

    try:
        import cv2

        cv2.setNumThreads(OPENCV_THREADS)
    except Exception:
        pass

    from tasks.path_task import DEFAULTS

    context = TaskContext(
        debug=args.debug or args.ascii,
        ascii_debug=args.ascii,
        use_lidar=True,
        use_camera=not args.no_camera,
        motor_port=args.motor_port,
        lidar_port=args.lidar_port,
        start_pose=args.start_pose,
        no_drive=args.no_drive,
        record_video=args.record or config.get("camera.record_video", False),
        lidar_ahead_mm=config.get("robot.lidar_ahead_mm",
                                  DEFAULTS["robot.lidar_ahead_mm"]),
    )

    with context:
        task = ParkingTestTask(context, config=config,
                               wall_side=1.0 if args.wall == "right" else -1.0,
                               max_runtime_s=max_runtime)
        return task.run()


if __name__ == "__main__":
    sys.exit(0 if main() else 1)

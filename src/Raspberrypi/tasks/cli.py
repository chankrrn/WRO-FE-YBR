import argparse
import sys
from pathlib import Path

from classes.task_context import TaskContext
from tasks.base_task import DEFAULT_MAX_RUNTIME_S
from utils.task_config import TaskConfig

# OpenCV defaults to one worker per core, and on a 4-core Pi 5 a round is
# already running three things that matter: the control loop, the lidar's
# scan thread, and the vision thread. Letting a colour threshold fan out over
# every core to save a fraction of a millisecond costs the control loop its
# scheduling latency, which is the thing being protected.
#
# The pipeline is cheap enough now that this is nearly free: measured on a
# 640x280 frame, crop + HSV + detect is 1.6ms single-threaded and 1.1ms on
# four. Two workers keep most of that and leave the loop a core.
OPENCV_THREADS = 2


def build_parser(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--laps", type=int, default=None,
                        help="laps to complete before stopping (default: from config.toml)")
    parser.add_argument("--speed", type=int, default=None,
                        help="override the round's straight-line speed")
    parser.add_argument("--corner-speed", type=int, default=None,
                        help="override the round's corner speed")
    parser.add_argument("--config", default=None,
                        help="tunables TOML (default: config.toml beside the round)")
    parser.add_argument("--debug", action="store_true",
                        help="verbose logging plus the navigation/camera debug windows")
    parser.add_argument("--ascii", action="store_true",
                        help="print the navigation debug readout as text instead of "
                             "opening a window, for plain SSH (implies --debug)")
    parser.add_argument("--lidar", action="store_true",
                        help="start the lidar even if the task does not require it")
    parser.add_argument("--no-camera", action="store_true",
                        help="skip the camera even if the task requires it")
    parser.add_argument("--no-drive", action="store_true",
                        help="never spin the drive motor: the steering servo still "
                             "follows the real control loop, so you can push the robot "
                             "round by hand and watch it steer (no time limit unless "
                             "--max-runtime says otherwise)")
    parser.add_argument("--max-runtime", type=float, default=None,
                        help="stop after N seconds (default: 180, the WRO limit; "
                             "no limit under --no-drive)")
    parser.add_argument("--motor-port", default=None,
                        help="Arduino serial port (default: autodetect)")
    parser.add_argument("--lidar-port", default=None,
                        help="lidar serial port (default: autodetect)")
    parser.add_argument("--start-pose", type=float, nargs=3, default=None,
                        metavar=("X_MM", "Y_MM", "HEADING_DEG"),
                        help="where the robot is placed, for context.nav: mm from the "
                             "middle of the field and degrees clockwise from +Y "
                             "(default: let the localizer search the field)")
    parser.add_argument("--dry-run", action="store_true",
                        help="build the path and print the plan without touching hardware")
    return parser


def load_config(task_class, args):
    """
    This round's tunables, with the CLI flags layered on top.

    The config file lives beside the round's task.py, so `--config` is only
    needed to try an alternative set of numbers without editing the real one.
    """
    if args.config:
        path = Path(args.config)
    else:
        path = Path(sys.modules[task_class.__module__].__file__).parent / "config.toml"

    config = TaskConfig.load(path)
    config.set("laps.goal", args.laps)
    config.set("speed.base", args.speed)
    config.set("speed.corner", args.corner_speed)
    return config


def run_task(task_class, argv=None):
    """
    Shared entry point for every tasks/<round>/run.py: parses the standard
    flags, loads the round's config, builds a TaskContext wired for that task,
    and runs it inside the context manager so the hardware is always shut down
    afterwards.
    """
    args = build_parser(f"Run the {task_class.name} round").parse_args(argv)
    config = load_config(task_class, args)
    # Pushing the robot round by hand takes as long as it takes; being cut off
    # at the WRO limit mid-lap is not what --no-drive is for.
    max_runtime = args.max_runtime
    if max_runtime is None:
        max_runtime = None if args.no_drive else DEFAULT_MAX_RUNTIME_S

    if args.dry_run:
        return dry_run(task_class, config)

    # Before anything opens the camera - see OPENCV_THREADS.
    try:
        import cv2

        cv2.setNumThreads(OPENCV_THREADS)
    except Exception:
        pass

    context = TaskContext(
        debug=args.debug or args.ascii,
        ascii_debug=args.ascii,
        use_lidar=task_class.requires_lidar or args.lidar,
        use_camera=task_class.requires_camera and not args.no_camera,
        laps_goal=config.get("laps.goal", 3),
        motor_port=args.motor_port,
        lidar_port=args.lidar_port,
        start_pose=args.start_pose,
        no_drive=args.no_drive,
    )

    with context:
        task = task_class(context, config=config, max_runtime_s=max_runtime)
        return task.run()


def dry_run(task_class, config):
    """
    Prints the path and the settings the round WOULD run with, without opening
    a serial port or spinning a motor. Use it to sanity-check a config change
    before putting the robot on the mat.
    """
    from classes.field_map import FieldMap
    from classes.pure_pursuit import PurePursuit
    from classes.racing_line import RacingLine
    from tasks.path_task import DEFAULTS

    def setting(key):
        return config.get(key, DEFAULTS.get(key))

    field = FieldMap()
    path = RacingLine(field_map=field,
                      wall_margin_mm=setting("path.wall_margin_mm"),
                      corner_radius_mm=setting("path.corner_radius_mm"),
                      resolution_mm=setting("path.resolution_mm"))
    pursuit = PurePursuit(wheelbase_mm=setting("pursuit.wheelbase_mm"),
                          max_road_wheel_deg=setting("pursuit.max_road_wheel_deg"))

    print(f"{task_class.name}: dry run")
    print(f"  config          {config.source or 'built-in defaults'}")
    print(f"  field           {field.field_size_mm:.0f}mm outer, "
          f"{field.inner_size_mm:.0f}mm block, {field.corridor_width_mm:.0f}mm corridor")
    print(f"  {path}")
    print(f"  lap length      {path.length / 1000:.2f}m x {setting('laps.goal')} laps "
          f"= {path.length * setting('laps.goal') / 1000:.1f}m")
    print(f"  clearances      {field.outer - path.half:.0f}mm to the wall, "
          f"{path.half - field.inner:.0f}mm to the block on the straights")
    print(f"  turning circle  robot {pursuit.min_turn_radius_mm:.0f}mm vs "
          f"path corner {path.corner_radius:.0f}mm "
          f"{'OK' if path.corner_radius >= pursuit.min_turn_radius_mm else 'TOO TIGHT'}")
    print(f"  speed           base {setting('speed.base')}, corner {setting('speed.corner')}, "
          f"lost {setting('speed.lost')}")
    print(f"  lookahead       {setting('pursuit.lookahead_base_mm'):.0f}mm "
          f"+ {setting('pursuit.lookahead_per_speed_mm')}/speed, clamped to "
          f"{setting('pursuit.lookahead_min_mm'):.0f}-{setting('pursuit.lookahead_max_mm'):.0f}mm")
    return True

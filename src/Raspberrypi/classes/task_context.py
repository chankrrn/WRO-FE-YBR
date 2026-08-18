import time

from classes.board_manager import BoardManager
from classes.compass_manager import CompassManager
from classes.light_sensor_manager import LightSensorManager
from classes.motor_manager import MotorManager
from classes.navigation_manager import NavigationManager
from classes.navigator import Navigator
from classes.ultra_servo_manager import UltraServoManager


class TaskContext:
    """
    Everything a task runs on: every piece of hardware, the navigator built on
    top of them, and the startup/shutdown order that ties them together.

    A task never constructs hardware itself - it gets a started TaskContext and
    talks to context.motor, context.compass, context.navigator and so on. That
    keeps "how the robot is wired" in one place and lets the qualification and
    final rounds share it.

    Optional subsystems are opt-in because they cost time and are not needed by
    every round:
        use_lidar   - LidarManager (RPLidar C1 on USB)
        use_camera  - CameraManager + ObjectSolver (final round pillars)

    Usage:
        with TaskContext(debug=True) as context:
            context.motor.forward(55)
            print(context.nav.get_pose())
    """

    def __init__(self, debug=False, ascii_debug=False, use_lidar=False, use_camera=False,
                 laps_goal=3, motor_port=None, lidar_port=None,
                 start_pose=None):
        self.debug = debug
        self.ascii_debug = ascii_debug
        self.use_lidar = use_lidar
        self.use_camera = use_camera
        self.start_pose = start_pose

        self.board = BoardManager(debug=debug)
        self.motor = MotorManager(port=motor_port, debug=debug)
        self.compass = CompassManager(debug=debug)
        self.ultra = UltraServoManager(self.board, debug=debug)
        self.lights = LightSensorManager(self.board, debug=debug)
        self.navigator = Navigator(self.compass, self.ultra, laps_goal=laps_goal, debug=debug)

        # Always constructed so context.nav.get_pose() is safe to call from any
        # task, but it only has anything to localize against once the lidar is
        # up - without it every pose comes back with confidence 0.
        self.nav = NavigationManager(compass=self.compass, debug=debug)

        # Built during startup only when asked for.
        self.lidar = None
        self.camera = None
        self.object_solver = None
        self._lidar_port = lidar_port

        self._gpio = None
        self.started = False

    # ========================================================================
    # LIFECYCLE
    # ========================================================================

    def startup(self):
        """
        Brings every subsystem up in dependency order (the board first - the
        servo, ultrasonic and light sensors all hang off it). Anything that
        fails here raises, except the compass and the optional subsystems,
        which warn and carry on degraded.
        """
        if self.started:
            return self

        steps = [("GPIO", self._init_gpio),
                 ("Expansion board", self.board.start),
                 ("Motor", self.motor.start),
                #  ("Ultrasonic servo", self.ultra.start),
                 ("Compass", self.compass.start)]
        if self.use_lidar:
            steps.append(("Lidar", self._init_lidar))
        if self.use_camera:
            steps.append(("Camera", self._init_camera))
        # Last: it needs the compass anchored and the lidar already spinning.
        steps.append(("Navigation", self._init_nav))

        print("=" * 70)
        for index, (name, init) in enumerate(steps, start=1):
            print(f"[{index}/{len(steps)}] {name}...")
            init()
            print("  OK")
        print("=" * 70)

        self.started = True
        return self

    def shutdown(self):
        """Safe to call twice, and safe to call after a failed startup."""
        print("\nShutting down...")
        self.motor.stop_and_close()
        # self.ultra.stop()
        self.nav.stop()
        if self.lidar is not None:
            self.lidar.stop()
            self.lidar = None
        if self.camera is not None:
            try:
                self.camera.release_video()
            except Exception:
                pass
            self.camera = None
        self.compass.stop()
        self.board.stop()
        self._cleanup_gpio()
        self.started = False
        print("Done\n")

    def __enter__(self):
        return self.startup()

    def __exit__(self, exc_type, exc, tb):
        self.shutdown()
        return False

    # ========================================================================
    # SUBSYSTEM SETUP
    # ========================================================================

    def _init_gpio(self):
        import RPi.GPIO as GPIO

        # A previous run that died without cleaning up leaves the pins claimed,
        # so clear them before taking them again. Warnings go off first, or a
        # clean boot logs "nothing to clean up" every single run.
        GPIO.setwarnings(False)
        try:
            GPIO.cleanup()
        except Exception:
            pass
        time.sleep(0.2)
        GPIO.setmode(GPIO.BCM)
        self._gpio = GPIO

    def _cleanup_gpio(self):
        if self._gpio is None:
            return
        try:
            self._gpio.cleanup()
        except Exception:
            pass
        self._gpio = None

    def _init_lidar(self):
        from classes.lidar_manager import LidarManager

        self.lidar = LidarManager(port=self._lidar_port, debug=self.debug)
        self.lidar.start()
        if not self.lidar.wait_until_ready(timeout=5.0):
            print("  WARNING: lidar returned little data - is it spinning?")

    def _init_nav(self):
        """
        Points the localizer at whatever hardware actually came up and seeds
        the particle cloud.

        With `start_pose` the filter knows where it is from the first scan.
        Without it, it searches the whole field - which works, but the map is
        90-degree symmetric, so if the compass is also dead it can settle into
        the wrong quadrant. Give it a start pose when you know one.
        """
        self.nav.lidar = self.lidar
        self.nav.compass = self.compass if self.compass.is_available else None
        if self.lidar is None:
            print("  no lidar - poses will report confidence 0")

        self.nav.start(*(self.start_pose or ()))
        pose = self.nav.get_pose()
        print(f"  pose: {pose}")

    def _init_camera(self):
        from Raspberrypi.camera_manager import CameraManager
        from classes.object_solver import ObjectSolver

        try:
            self.camera = CameraManager()
            self.camera.start_camera()
            self.object_solver = ObjectSolver(debug=self.debug)
        except Exception as e:
            self.camera = None
            self.object_solver = None
            print(f"  WARNING: camera unavailable ({e!r}) - continuing without it")

    # ========================================================================
    # SHARED HELPERS
    # ========================================================================

    def wait_for_start(self):
        """Blocks on the physical start button, wired to the Arduino."""
        self.motor.wait_for_start()

    def emergency_stop(self):
        try:
            self.motor.stop()
            self.motor.steer_center()
        except Exception:
            pass

"""
The parking manoeuvre on its own, with no lap and no pillars.

WHY THIS EXISTS. Every mat run of the park so far has cost a lap and a half of
driving before the interesting part started, and every failure has had to be
untangled from whatever the lap did on the way - a pillar dodged late, a pose
that drifted, a wall gap the planner compromised on. None of that is the park.
This puts the robot down beside the bay, waits for the button, and runs
VelozParking from tick one against nothing else at all.

The obstacle round is untouched and still lives in tasks/final - that is where
the pillars, the racing line and the lap counting belong. This shares the
controller with it and nothing else, so a number tuned here is a number tuned
there.

HOW TO USE IT. Put the robot on the mat where the last lap would have left it:
beside the outer wall, a bay's length or so before the bay, pointing the way it
would be driving. Say which side the wall is on, because there is no lap here
to work it out from:

    uv run python -m tasks.parking_test.run --wall right
    uv run python -m tasks.parking_test.run --wall left --debug

Unlike the round, SEARCHING drives here (search_drives=True): there is no lap
to hand the wheels to, so the park rolls forward itself until the camera picks
up the bay, exactly as the original does.
"""
from classes.veloz_parking import VelozParking
from tasks.base_task import LOOP_PERIOD_S, Task
from utils.task_config import TaskConfig


class ParkingTestTask(Task):
    """
    One park, start to finish, and then stop.

    Never touches the motor through anything but the controller's own command,
    so what the wheels are told here is what they would be told in the round.
    """

    name = "parking_test"
    requires_lidar = True
    requires_camera = True

    def __init__(self, context, config=None, wall_side=1.0, **kwargs):
        super().__init__(context, **kwargs)
        self.config = config or TaskConfig({})
        self.wall_side = float(wall_side)
        self.parking = None
        self._last_phase = None

    def setting(self, key, default=None):
        return self.config.get(key, default)

    # ------------------------------------------------------------------
    def setup(self):
        self.context.motor.steer_center()
        if self.context.vision is not None:
            self.context.vision.watch_for_parking(True)

        self.parking = VelozParking(
            lidar=self.context.lidar,
            compass=self.context.compass,
            vision=self.context.vision,
            wall_side=self.wall_side,
            # No lap to hand the wheels to - the park does its own searching.
            search_drives=True,
            camera_confirms=bool(self.setting("parking.camera_confirms", True)),
            align_stop_mm=float(self.setting("parking.align_stop_mm", 300.0)),
            reverse_stop_mm=float(self.setting("parking.reverse_stop_mm", 150.0)),
            min_reverse_mm=float(self.setting("parking.min_reverse_mm", 80.0)),
            max_reverse_mm=float(self.setting("parking.max_reverse_mm", 480.0)),
            straighten_extra_mm=float(
                self.setting("parking.straighten_extra_mm", 120.0)),
            in_bay_mm=float(self.setting("parking.in_bay_mm", 220.0)),
            in_bay_ticks=int(self.setting("parking.in_bay_ticks", 5)),
            in_bay_after=float(self.setting("parking.in_bay_after", 0.7)),
            marker_hold_s=float(self.setting("parking.marker_hold_s", 0.4)),
            marker_lost_s=float(self.setting("parking.marker_lost_s", 0.8)),
            min_marker_area=float(
                self.setting("parking.marker_area_px", 1200.0)),
            abeam_bearing_deg=float(
                self.setting("parking.abeam_bearing_deg", 40.0)),
            past_bay_mm=float(self.setting("parking.past_bay_mm", 150.0)),
            approach_max_mm=float(
                self.setting("parking.approach_max_mm", 1200.0)),
            approach_wall_gain=float(
                self.setting("parking.approach_wall_gain", 0.05)),
            approach_max_steer=float(
                self.setting("parking.approach_max_steer", 25.0)),
            approach_blade_mm=float(
                self.setting("parking.approach_blade_mm", 120.0)),
            trigger_side_mm=float(
                self.setting("parking.trigger_side_mm", 900.0)),
            speed=int(self.setting("parking.speed", 55)),
            reverse_speed=int(self.setting("parking.reverse_speed", 60)),
            mm_per_s_at_full=float(
                self.setting("startup.mm_per_s_at_full", 390.0)),
            timeout_s=float(self.setting("parking.timeout_s", 20.0)))

        side = "right" if self.wall_side > 0 else "left"
        print(f"\nParking test - outer wall on the {side}.")
        print(f"  {self.parking.summary()}")
        print("  Place the robot beside the wall, short of the bay, pointing "
              "the way the lap would be going.\n")

    # ------------------------------------------------------------------
    def step(self):
        # The pose is passed as None on purpose: VelozParking never reads it,
        # and this bench test should not need a localizer to be up.
        command = self.parking.update(None, LOOP_PERIOD_S)
        if command is None:
            self.context.motor.drive(0, 0)
            return
        steer, speed = command
        self.context.motor.drive(steer, speed)

        # One line per state change, so the transcript of a run is the list of
        # states it went through and what the ranges were as it did.
        if self.parking.phase != self._last_phase:
            print(f"  -> {self.parking.status_line()}")
            self._last_phase = self.parking.phase

    def is_finished(self):
        return self.parking is not None and self.parking.finished

    def finish(self):
        self.context.motor.stop()
        self.context.motor.steer_center()
        if self.parking is None:
            return
        if self.parking.phase == VelozParking.DONE:
            print(f"\nPARKED after {self.parking.reversed_mm:.0f}mm of reverse.")
        else:
            print(f"\nDID NOT PARK: {self.parking.reason or self.parking.phase}")

    def status(self):
        if self.parking is None:
            return "starting"
        return self.parking.status_line()

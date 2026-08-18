"""
Obstacle round: the same three laps, but each pillar has to be passed on the
side its colour dictates.

    GREEN -> the robot passes on the block's LEFT
    RED   -> the robot passes on the block's RIGHT

Nothing about the base lap-following changes here - steering, lookahead,
speed and lidar safety are all inherited untouched from PathDrivingTask (see
tasks/path_task.py). This file answers exactly one question,
`target_lateral_mm()`: how far sideways the racing line should sit at a given
point on the lap, so pure pursuit bends around a pillar the same way it
follows any other point on the track. Everything else here exists to compute
that one number correctly.
"""
import cv2
import numpy as np

from classes.block_map import BLOCK_SIZE_MM
from tasks.path_task import PathDrivingTask
from utils.angle_utils import clamp
from utils.enums import Color

# `lateral` is positive to the right of travel (see RacingLine.project).
# Passing a block on its LEFT means the robot ends up to the left of it,
# i.e. a negative offset from the block's own position.
SIDE_FOR_COLOR = {Color.GREEN: -1.0, Color.RED: +1.0}

CAMERA_EVERY_N_TICKS = 2


def _smoothstep(t):
    """
    Eases 0->1 with zero slope at both ends, instead of a straight ramp.

    A linear ramp changes the lateral offset at a CONSTANT rate, so the
    target point pure pursuit chases moves in a straight diagonal line and
    kinks sharply where the ramp starts and stops. This starts and ends the
    sweep at zero rate instead, so the line curves smoothly into and out of
    the dodge rather than cornering at the joints.
    """
    t = clamp(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


class FinalTask(PathDrivingTask):
    """
    PathDrivingTask with the racing line bent sideways around each pillar.

    Rather than a separate avoidance controller fighting the path follower,
    the pillar just moves the line: `target_lateral_mm` returns where the
    line should sit at a given point on the lap, and pure pursuit follows it
    as it would follow anything else.

    Because the offset is a function of PROGRESS, not of what is in shot, the
    robot starts easing across well before it reaches a pillar and is already
    on the correct side when it arrives - and it keeps doing so for pillars
    the camera has since looked away from, because they live in nav.blocks,
    not in the current frame.
    """

    name = "final"
    requires_camera = True

    def setup(self):
        super().setup()
        if self.context.object_solver is None:
            print("WARNING: no camera - the final round will drive the plain "
                  "racing line and ignore the pillars")
        self._warn_if_dodge_wont_fit()

    def _warn_if_dodge_wont_fit(self):
        """
        Checks the worst case - a pillar sitting exactly ON the racing line -
        against how much room wall_clearance_mm actually leaves, and says so
        if they do not fit.

        Without this, a dodge that is geometrically too big for the corridor
        does not fail loudly: `target_lateral_mm` just clamps it to whatever
        fits, silently delivering less than clearance_mm of real air gap.
        That clamp is the right behaviour (never dodge into a wall to give a
        pillar more room), but finding out about it live, mid-collision, is
        not - this says so at startup instead, in plain mm.
        """
        limit = self._wall_limit_mm()
        for color in (Color.RED, Color.GREEN):
            needed = self._required_offset_mm(color)
            if needed > limit:
                print(f"WARNING: a {color.value} pillar sitting on the racing "
                      f"line would need a {needed:.0f}mm dodge, but "
                      f"wall_clearance_mm only leaves {limit:.0f}mm of room - "
                      f"clearance_mm will be delivered short for pillars near "
                      f"the line. Lower blocks.clearance_mm / "
                      f"blocks.robot_half_width_mm, or raise "
                      f"blocks.wall_clearance_mm.")

    def step(self):
        # The camera pipeline costs far more than a control tick, so detection
        # runs on its own slower cadence. The block map is what the steering
        # reads, and that persists between frames.
        if self.tick % CAMERA_EVERY_N_TICKS == 0:
            self._update_detections()
        super().step()

    def _update_detections(self):
        context = self.context
        if context.object_solver is None or context.camera is None:
            return
        try:
            context.camera.capture_image()
            context.camera.transform_image()
            context.nav.observe_blocks(context.object_solver.detect(
                context.camera.hsv_image, display_image=context.camera.display_image))
            # transform_image() is what fills in display_image - nothing else
            # in the round ever calls this, so without it release_video() was
            # finalizing an empty file every run.
            # context.camera.add_frame_to_video()
        except Exception as e:
            print(f"WARNING: detection failed: {e!r}")

    # ========================================================================
    # PILLARS
    # ========================================================================

    def _required_offset_mm(self, color):
        """
        How far from the PILLAR'S OWN position the robot's steered centerline
        must sit for its body to actually clear it by clearance_mm.

        Three parts, all real mm: clearance_mm is the gap you want between
        the robot's body and the pillar's face; robot_half_width_mm converts
        that from a body-relative gap into a centerline-relative one, since
        pure pursuit steers the centerline, not the edge; BLOCK_SIZE_MM/2
        reaches from the pillar's own center out to its near face. Colour
        extras are an optional, independent pad per colour - see
        blocks.red_extra_clearance_mm / blocks.green_extra_clearance_mm.
 _smoothstep(t):
    _smoothstep(t):
    _smoothstep(t):
           """
        extra_key = {Color.RED: "blocks.red_extra_clearance_mm",
                     Color.GREEN: "blocks.green_extra_clearance_mm"}[color]
        return (float(self.setting("blocks.clearance_mm"))
               + float(self.setting("blocks.robot_half_width_mm"))
               + float(self.setting(extra_key))
               + BLOCK_SIZE_MM / 2.0)

    def _wall_limit_mm(self):
        """
        How far the steered centerline may move off the racing line before
        the robot's BODY would be closer than wall_clearance_mm to the outer
        wall or the centre block - whichever the corridor runs out of first.

        Same body-vs-centerline correction as _required_offset_mm: without
        adding robot_half_width_mm back in, this caps the centerline's
        distance from the wall, not the body's, and the body ends up
        wall_clearance_mm short of the real wall by however wide the chassis
        is.
        """
        wall = (float(self.setting("blocks.wall_clearance_mm"))
               + float(self.setting("blocks.robot_half_width_mm")))
        return self.path.lateral_limit(wall)

    def _nearest_actionable_block(self, blocks, progress):
        """
        Which confirmed block, if any, should be steering the line right now.

        Candidates are confirmed blocks within [-past_mm, approach_mm] of the
        current progress. Among those, a block still AHEAD always outranks
        one already behind, however close the trailing one is - otherwise a
        just-passed pillar can keep winning "nearest" on raw distance and eat
        into the ramp time the NEXT pillar needed to be dodged in time. Within
        the same ahead/behind group, nearest wins outright rather than
        blending several: averaging two pillars that want opposite sides
        steers between them, into both.

        I/O:
            return: (block, gap, block_lateral) for the winner, or None
        """
        approach = float(self.setting("blocks.approach_mm"))
        past = float(self.setting("blocks.past_mm"))

        best = None
        for block in blocks:
            if block.color not in SIDE_FOR_COLOR:
                continue
            block_progress, block_lateral = self.path.project(
                block.x, block.y, self.direction)
            gap = self.path.gap(progress, block_progress)
            if not (-past <= gap <= approach):
                continue

            ahead = gap >= 0.0
            best_ahead = best is not None and best[1] >= 0.0
            if (best is None or (ahead and not best_ahead)
                    or (ahead == best_ahead and abs(gap) < abs(best[1]))):
                best = (block, gap, block_lateral)
        return best

    def target_lateral_mm(self, progress):
        """
        Where the racing line should sit at `progress`, to pass the pillars
        correctly.

        The winning block (see _nearest_actionable_block) is projected onto
        the path, and the line is pulled to THAT block's own lateral
        position, offset by _required_offset_mm to the required side - so the
        robot dodges relative to where the pillar actually is, not by a fixed
        amount from the centre line. A pillar already sitting wide gets a
        smaller correction; one on the racing line gets the full offset.

        The pull fades in and out smoothly (see _smoothstep) rather than
        stepping, and is clamped by _wall_limit_mm so it can never push the
        robot's body closer than wall_clearance_mm to the wall or the centre
        block, however large clearance_mm asks for - see
        _warn_if_dodge_wont_fit for when that clamp is actually binding.
        """
        blocks = self.context.nav.blocks.confirmed()
        if not blocks:
            return 0.0

        target = self._nearest_actionable_block(blocks, progress)
        if target is None:
            return 0.0
        block, gap, block_lateral = target

        approach = float(self.setting("blocks.approach_mm"))
        past = float(self.setting("blocks.past_mm"))
        ramp_in = _smoothstep((approach - gap) / max(1.0, approach * 0.5))
        ramp_out = _smoothstep((gap + past) / max(1.0, past * 0.5))
        ramp = min(ramp_in, ramp_out)

        side = SIDE_FOR_COLOR[block.color]
        wanted = block_lateral + side * self._required_offset_mm(block.color)
        limit = self._wall_limit_mm()
        return clamp(wanted * ramp, -limit, limit)

    # ========================================================================
    # REPORTING
    # ========================================================================

    def status(self):
        return f"{super().status()}  {self.context.nav.blocks.summary()}"

    def _draw_overlay(self, canvas, to_px):
        """The plain racing line plus the bent one the robot is actually on."""
        super()._draw_overlay(canvas, to_px)

        points = []
        for step in range(0, int(self.path.length), 40):
            progress = self.progress + step
            x, y = self.path.point_at(progress, self.direction,
                                      self.target_lateral_mm(progress))
            points.append(to_px(x, y))
        if points:
            cv2.polylines(canvas, [np.array(points, dtype=np.int32)],
                          isClosed=True, color=(180, 140, 60), thickness=1)

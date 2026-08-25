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
import math
import time
from collections import namedtuple

import cv2
import numpy as np

from classes.block_map import BLOCK_SIZE_MM
from classes.parking import (WALL_LENGTH_MM, BayFinder, BayFrame,
                             ParkingController, bay_pose, section_of,
                             wall_rects)
from tasks.path_task import PathDrivingTask
from utils.angle_utils import clamp
from utils.enums import Color

# `lateral` is positive to the right of travel (see RacingLine.project).
# Passing a block on its LEFT means the robot ends up to the left of it,
# i.e. a negative offset from the block's own position.
SIDE_FOR_COLOR = {Color.GREEN: -1.0, Color.RED: +1.0}

CAMERA_EVERY_N_TICKS = 2

# One pillar's claim on the line: how far sideways it wants the line (`target`,
# absolute, in the travel frame) and how much of that claim applies right now
# (`weight`, 0-1). `block` and `gap` are carried along for the status line -
# "the camera can see it" and "the line is acting on it" are different
# questions, and the second one is the one that is hard to see from outside.
# One point on the lap's offset profile. The profile is nothing but a list of
# these joined by cubic Beziers, which is what makes it continuous by
# construction rather than by arithmetic that has to be checked - see
# FinalTask._build_line.
Node = namedtuple("Node", "progress offset")

# One stretch between two nodes, ready to evaluate: where it starts, how long
# it is, and the offsets it runs between.
Span = namedtuple("Span", "start length begin end")

# Where a transition's two inner control points sit along it, as a fraction of
# its length. Both ends of the curve are horizontal whatever these are - a
# cubic Bezier's tangent at P0 points at P1, and P1 sits level with P0 - so the
# handles do not decide WHETHER the line joins smoothly, only how much of the
# transition is spent turning:
#
#     0.33   the smoothstep this used to be, exactly
#     lower  leaves the straight sooner and turns more evenly - gentler in the
#            middle, sharper where it joins
#     higher holds the straight longer and turns harder in the middle
#
# The peak lean of a symmetric transition works out at offset / (length x
# (1 - handle)), so 0.33 leans at 1.5 x offset / length and 0.5 at 2x.
DEFAULT_HANDLE = 1.0 / 3.0

# The least of the gap between two pillars that each of their flat parts keeps
# when the curve joining them cannot be given the length it wants - see
# _share_gap. Also how much of a late pillar's whole sighting distance goes to
# its flat part rather than to easing into it.
MIN_PADDING_SHARE = 0.1

# How much clear lap the competition guarantees either side of a parking
# space: nothing is placed nearer to one than this. It is what lets the bay
# guard cover only the bay instead of the whole section it sits in - wherever
# the bay turns out to be, no pillar's pass is beside it, so the line is on or
# near the racing line there and already clear of a bay wall. See
# FinalTask._bay_wall_limit_mm.
PILLAR_FREE_MM = 500.0

# How finely the corridor is measured along a dodge's whole reach when working
# out how much of it will fit. The features it has to catch - a corner arc, the
# parking bay's window - are hundreds of mm wide.
BOUNDS_SAMPLE_MM = 50.0

# How far the profile has to move at the pursuit target before it counts as a
# jump worth easing rather than an ordinary refinement worth following. A
# pillar arriving moves it by most of a dodge; the map nudging one that is
# already there moves it by a millimetre or two.
CATCHUP_DEADBAND_MM = 10.0

# How much of a dodge a corner may quietly take before setup says so loudly.
# On a well-proportioned line the corner is a few mm tighter than the straights
# and the shortfall is not worth a WARNING; it is when the arcs are small
# enough to eat a real part of the clearance that it matters.
CORNER_CUT_TOLERANCE = 0.1

# Two sightings this far apart along the lap are the same pillar. Ramp anchors
# are remembered against a POSITION on the track rather than against a tracked
# block, because a block is not a stable thing to hang anything on: lose it for
# MAX_MISSES frames and BlockMap drops it, and the re-detection that follows is
# a new track with a new uid and no history. Comfortably wider than BlockMap's
# own ASSOCIATION_RADIUS_MM (180mm), since anything closer than this along the
# lap could not be dodged as a separate pillar anyway.
ANCHOR_SAME_PILLAR_MM = 200.0

# One remembered pillar: where it is (`progress`, `lateral`), which way to
# pass it (`color`), how early it was first seen (`anchor`), and when it was
# last actually confirmed.
#
# The line is steered off THIS, not off nav.blocks.confirmed() directly. A
# dodge recomputed from live confirmations every tick does not degrade when a
# frame is missed, it is DELETED - the offset snaps to centre and back out
# again the moment BlockMap drops a track, which at range it does readily,
# because a pillar 2m off is a handful of pixels and MAX_MISSES is six frames.
# That flicker lands on the approach, where the pillar is furthest and the
# detection worst, and never on the exit, where it is close and solid.
Pillar = namedtuple("Pillar", "progress lateral color anchor last_seen")


def _bezier_t(x, lead, settle):
    """
    The curve parameter at along-track fraction `x` of a transition.

    A cubic Bezier is parametric - moving `t` evenly does not move evenly along
    the track - so reading the offset at a given point on the lap means solving
    for `t` first. Newton on a well-behaved monotonic cubic, from a starting
    guess that is already close; four passes take it to well under a
    millimetre, and there is nothing here that can fail to converge because the
    handles are clamped to keep the curve monotonic along the track.
    """
    t = clamp(x, 0.0, 1.0)
    for _ in range(4):
        one = 1.0 - t
        # B(t) with control points 0, lead, 1 - settle, 1
        value = (3.0 * one * one * t * lead
                 + 3.0 * one * t * t * (1.0 - settle)
                 + t * t * t)
        slope = 3.0 * (one * one * lead
                       + 2.0 * one * t * (1.0 - settle - lead)
                       + t * t * settle)
        if slope <= 1e-9:
            break
        t = clamp(t - (value - x) / slope, 0.0, 1.0)
    return t


def _bezier_ease(fraction, lead, settle):
    """
    Eases 0 -> 1 across a transition, as a cubic Bezier with both ends
    horizontal.

    The four points the curve is drawn through are (0, 0), (lead, 0),
    (1 - settle, 1) and (1, 1), in units of the transition's own length. The
    two middle ones are the handles: they sit level with the ends, which is
    what makes the line leave and arrive parallel to the racing line, and their
    positions ALONG the transition are the tuning.

    With both handles at 1/3 this is exactly 3t^2 - 2t^3, the smoothstep the
    profile used before, so leaving them alone changes nothing.
    """
    t = _bezier_t(clamp(fraction, 0.0, 1.0), lead, settle)
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

    def __init__(self, context, config=None, **kwargs):
        super().__init__(context, config=config, **kwargs)
        # Pillars being steered around, held across detection dropouts.
        # See _update_pillar_memory.
        self._pillars = []
        # The lap's offset profile, rebuilt each tick - see _build_line.
        self._line = []
        self._wanted_line = []
        # How far the line currently sits from the profile, and the point on
        # the lap where that gap applies in full - see _settle_bias.
        self._bias = 0.0
        self._bias_from = 0.0
        # The parking bay: which section it is in, where along that section,
        # and where that lands on the lap. None until it has been found.
        self._start_section = None
        self._bay = None
        self._bay_progress = None
        self._bay_finder = None
        self._parking = None

    def setup(self):
        super().setup()
        if self.context.object_solver is None:
            print("WARNING: no camera - the final round will drive the plain "
                  "racing line and ignore the pillars")
        self._apply_map_range()
        pose = self.context.nav.get_pose()
        self._start_section = section_of(pose.x, pose.y, self.context.nav.map)
        print(f"Parking bay expected in the {self._start_section} section")
        if self.setting("parking.enabled") and self._start_section is not None:
            self._bay_finder = BayFinder(
                self.context.nav.map, self._start_section,
                min_depth_mm=float(self.setting("parking.detect_min_depth_mm")),
                min_gap_mm=float(self.setting("parking.detect_min_gap_mm")),
                max_gap_mm=float(self.setting("parking.detect_max_gap_mm")),
                min_scans=int(self.setting("parking.detect_min_scans")))
        self._warn_if_dodge_wont_fit()
        self._warn_if_approach_is_out_of_reach()
        self._warn_if_corners_wont_take_the_dodge()
        self._warn_if_the_bay_guard_blocks_a_dodge()
        self._warn_if_transitions_are_too_short()

    def _apply_map_range(self):
        """
        Lets this round push the range at which a pillar gets mapped at all,
        because that - not approach_mm - is what really decides how early the
        line can start moving.

        block_map's own MAX_MAPPING_RANGE_MM stays the default and the reason
        for it still holds: ObjectSolver ranges a pillar off its apparent
        height, so the error grows with the square of distance and a far
        pillar lands on the map in roughly the right direction but not the
        right place. What makes it worth raising ANYWAY is the shape of the
        ramp - a smoothstep leaves the far end with zero slope, so the first
        part of the sweep, which is the part fed by the worst position
        estimate, moves the line barely at all, and the estimate has tightened
        up by the time the offset actually matters. The cost that does not
        wash out is a false positive at range: a red smudge on the wall now
        gets a vote on the line 2m out instead of being ignored.
        """
        reach = self.setting("blocks.map_range_mm")
        if not reach:
            return
        blocks = self.context.nav.blocks
        print(f"Pillar mapping range: {blocks.max_range_mm:.0f}mm -> {float(reach):.0f}mm "
              f"(blocks.map_range_mm)")
        blocks.max_range_mm = float(reach)

    def _sweep_reach_mm(self):
        """
        How far ahead a sweep can actually start, given where pillars appear.

        Three subtractions, all real: a pillar is mapped from the LENS, which
        sits behind the robot's centre; the ramp is measured at the pursuit
        target, which is a lookahead in FRONT of it; and HITS_TO_CONFIRM
        sightings have to land before any of it counts, which at this speed is
        a few more centimetres. The last one is left out here - it varies with
        frame rate - so this is the optimistic figure.
        """
        forward_offset = self.context.nav.blocks.camera_offset_mm[0]
        lookahead = self.pursuit.lookahead_distance(float(self.setting("speed.base")))
        return self.context.nav.blocks.max_range_mm + forward_offset - lookahead

    def _warn_if_approach_is_out_of_reach(self):
        """
        Says so when approach_mm is asking for a sweep that cannot start that
        early, whatever it is set to.

        approach_mm is a CAP on how early the line starts moving, not a
        promise - _ramp_start takes the earliest of it, where the pillar was
        really first seen, and where the pillar in front of it was passed. Set
        it past what the camera can deliver and it simply stops being the
        limit that decides anything, which from the outside looks exactly like
        the config file not being read.
        """
        approach = (float(self.setting("blocks.padding_mm"))
                    + float(self.setting("blocks.approach_mm")))
        reach = self._sweep_reach_mm()
        if approach <= reach:
            return
        print(f"WARNING: blocks.padding_mm + approach_mm is {approach:.0f}mm, but a pillar "
              f"is not on "
              f"the map until it is within {self.context.nav.blocks.max_range_mm:.0f}mm of "
              f"the lens, so a sweep cannot start earlier than about {reach:.0f}mm out - "
              f"raising approach_mm above that changes nothing. To start it earlier, "
              f"raise blocks.map_range_mm.")

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

    def _warn_if_transitions_are_too_short(self):
        """
        Says how hard the line will have to lean, in degrees, for the dodge
        these settings ask for - both easing into a lone pillar and crossing
        between two that want opposite sides.

        Neither is ever traded away to make the other gentler: the dodge is
        delivered and the lean is whatever it has to be, because a steep dodge
        gets past a pillar and a small one does not. But past about 45 degrees
        pure pursuit stops following the line and starts cutting across it, and
        the number that decides this is not a transition length - it is
        clearance_mm. A 305mm dodge has to cross 610mm of corridor between an
        adjacent green and red however much lap there is to do it in, so a
        pair 600mm apart is steep no matter what approach_mm says.
        """
        offset = max(self._required_offset_mm(color)
                     for color in (Color.RED, Color.GREEN))
        lead, settle = self._handles()
        rise = 1.0 - (lead + settle) / 2.0

        for key, share in (("blocks.approach_mm", 1.0 - lead),
                           ("blocks.past_mm", 1.0 - settle)):
            length = float(self.setting(key))
            if length > 0.0:
                lean = math.degrees(math.atan(offset / (length * max(share, 1e-6))))
                print(f"{key} = {length:.0f}mm eases the {offset:.0f}mm dodge in at "
                      f"{lean:.0f} deg at its steepest")

        print(f"Two pillars wanting opposite sides need the line to cross "
              f"{2 * offset:.0f}mm, which at {self.setting('blocks.max_lean_deg'):.0f} deg "
              f"takes {2 * offset / max(rise, 1e-6) / math.tan(math.radians(float(
                  self.setting('blocks.max_lean_deg')))):.0f}mm of lap:")
        for spacing in (600.0, 800.0, 1000.0, 1400.0):
            padding = max(spacing * MIN_PADDING_SHARE,
                          min(float(self.setting("blocks.padding_mm")),
                              (spacing - 2 * offset / max(rise, 1e-6)) / 2.0))
            curve = max(spacing - 2 * padding, 1e-6)
            lean = math.degrees(math.atan(2 * offset / (curve * rise)))
            flag = "" if lean <= 50.0 else "   <- pure pursuit will cut this"
            print(f"    {spacing:.0f}mm apart -> {lean:.0f} deg{flag}")
        print(f"  (lower blocks.clearance_mm to bring these down - it is the only "
              f"thing that does; the dodge is {offset:.0f}mm because clearance_mm is "
              f"{float(self.setting('blocks.clearance_mm')):.0f}mm)")

    def _warn_if_the_bay_guard_blocks_a_dodge(self):
        """
        Says what the parking-bay guard will and will not take out of a dodge,
        at startup, in mm.

        This is the least obvious limit in the round: it is not in [blocks] at
        all, it applies to part of one section, and from the driver's seat it
        looks like the line simply refusing to go near the wall.
        """
        if self._start_section is None:
            return
        speculative = bool(self.setting("parking.guard_before_found"))
        needed = max(self._required_offset_mm(color)
                     for color in (Color.RED, Color.GREEN))
        clearance = (WALL_LENGTH_MM
                     + float(self.setting("parking.wall_margin_mm"))
                     + float(self.setting("blocks.robot_half_width_mm")))
        limit = max(0.0, self.path.map.outer - self.path.half - clearance)

        if not speculative:
            print(f"Parking-bay guard: {limit:.0f}mm toward the wall within "
                  f"{float(self.setting('parking.window_mm')):.0f}mm of the bay once it is "
                  f"found, and nowhere else - the field never puts a pillar within "
                  f"{PILLAR_FREE_MM:.0f}mm of a parking space, so no pass is beside it. "
                  f"Pillar dodges get the full corridor everywhere "
                  f"(parking.guard_before_found)")
            return

        where = f"toward the outer wall across the whole {self._start_section} section"
        if needed <= limit:
            print(f"Parking-bay guard leaves {limit:.0f}mm {where} until the bay is "
                  f"found - enough for the {needed:.0f}mm dodge")
            return
        body = (limit - float(self.setting("blocks.robot_half_width_mm"))
                - BLOCK_SIZE_MM / 2.0)
        print(f"WARNING: parking.guard_before_found is on, so until the bay is found "
              f"only {limit:.0f}mm is allowed {where} - but a pillar passed on that "
              f"side needs {needed:.0f}mm, so it will be passed at {limit:.0f}mm, i.e. "
              f"{body:.0f}mm of body gap. The field never puts a pillar within "
              f"{PILLAR_FREE_MM:.0f}mm of a parking space, so on a competition mat this "
              f"is protecting against a case that cannot arise - turn it off unless "
              f"this layout puts a pillar beside the bay.")

    def _start_section_progress(self):
        """A progress that lies in the starting section - what the bay guard is
        asked about when reporting what it will allow."""
        for step in range(0, int(self.path.length), 50):
            x, y, _ = self.path.pose_at(float(step), self.direction)
            if section_of(x, y, self.context.nav.map) == self._start_section:
                return float(step)
        return 0.0

    def _warn_if_corners_wont_take_the_dodge(self):
        """
        Says how much of a dodge the corners will take, in plain mm, at
        startup.

        This is the one limit that is invisible from the config file: nothing
        in [blocks] mentions the corner radius, and nothing in [path] mentions
        the dodge, but a line bent inward through an arc of radius R is an arc
        of radius R - offset, so path.wall_margin_mm silently decides how big
        an inward dodge can be. On the current geometry that limit is tighter
        than clearance_mm asks for, and a pillar in a corner gets a dodge cut
        to fit rather than the one that was configured.
        """
        room = self._corner_room_mm()
        if room == float("inf"):
            return
        worst = max(self._required_offset_mm(color)
                    for color in (Color.RED, Color.GREEN))
        detail = (f"corner radius {self.path.corner_radius:.0f}mm, turning circle "
                  f"{self.pursuit.min_turn_radius_mm:.0f}mm x "
                  f"{float(self.setting('blocks.turn_radius_margin')):.2f}")
        if worst <= room * (1.0 + CORNER_CUT_TOLERANCE):
            short = max(0.0, worst - room)
            print(f"Corners take {'the full' if short <= 0.5 else f'{room:.0f}mm of the'} "
                  f"{worst:.0f}mm dodge ({room:.0f}mm of inward room; {detail})")
            return
        print(f"WARNING: a pillar in a CORNER needing to be passed on the inside "
              f"wants a {worst:.0f}mm dodge, but bending the line inward that far "
              f"would make it tighter than the robot can steer - only {room:.0f}mm "
              f"is drivable ({detail}), so such a pillar gets a dodge cut to "
              f"{room / worst * 100:.0f}%, i.e. "
              f"{room - float(self.setting('blocks.robot_half_width_mm')) - BLOCK_SIZE_MM / 2.0:.0f}mm "
              f"of real body clearance instead of "
              f"{float(self.setting('blocks.clearance_mm')):.0f}mm. Lower "
              f"blocks.clearance_mm, or LOWER path.wall_margin_mm - moving the "
              f"racing line closer to the outer wall is what widens the corner "
              f"arcs, and the arcs are what this is short of.")

    def step(self):
        # Detection normally runs on VisionManager's thread and this does
        # nothing - the steering reads nav.blocks, which persists between
        # frames, not the current frame. The inline path is the fallback for
        # when the thread could not be started at all; it is what the round
        # used to do every other tick, and it costs the tick it runs on.
        if self.context.vision is None:
            if self.tick % CAMERA_EVERY_N_TICKS == 0:
                self._update_detections()
        super().step()

    def _update_detections(self):
        context = self.context
        if context.object_solver is None or context.camera is None:
            return
        try:
            # capture_for_blocks(), not capture_image() + transform_image():
            # this round reads the HSV frame and nothing else, and the full
            # pipeline costs ~19ms a frame to produce ~1.6ms of answer. The
            # display copy is only worth taking when the solver is going to
            # draw on it.
            hsv = context.camera.capture_for_blocks(
                with_display=context.object_solver.debug)
            if hsv is None:
                return
            context.nav.observe_blocks(context.object_solver.detect(
                hsv, display_image=context.camera.display_image))
        except Exception as e:
            print(f"WARNING: detection failed: {e!r}")

    # ========================================================================
    # PILLARS
    # ========================================================================

    def _track_progress(self, pose):
        """Where we are, as the base round tracks it, plus the pillar memory."""
        super()._track_progress(pose)
        self._update_pillar_memory()
        self._look_for_bay(pose)

    # ========================================================================
    # PARKING
    # ========================================================================

    def _look_for_bay(self, pose):
        """
        Watch for the bay on EVERY lap, not just the last one.

        The bay does not move and the robot goes past it three times, so
        accumulating sightings turns the parking lap into "drive to a known
        place" instead of "find it now, with the clock running". Stops as soon
        as it is found.
        """
        if self._bay_finder is None or self._bay is not None:
            return
        lidar = self.context.lidar
        if lidar is None or section_of(pose.x, pose.y, self.context.nav.map) != self._start_section:
            return
        bay = self._bay_finder.observe(pose, lidar.get_scan())
        if bay is None:
            return

        self._bay = bay
        x, y = self._bay_finder.bay_point()
        self._bay_progress, _ = self.path.project(x, y, self.direction)
        # Now that the walls are known, put them on the map the filter matches
        # against. Until this point every beam that hit one was unexplained,
        # which is exactly the wrong thing to be happening as the robot lines
        # up to reverse between them. Raycast-only - see FieldMap.add_obstacle.
        self.context.nav.map.set_obstacles(
            wall_rects(bay[0], bay[1], self.context.nav.map,
                       bay_mm=self._bay_finder.bay_mm))
        print(f"Found the bay: {self._bay_finder.status_line()} "
              f"(lap {self.laps_done:.2f})")

    def _start_parking(self):
        """Builds the manoeuvre once the laps are done and the bay is known."""
        _, _, heading = self.path.pose_at(self._bay_progress, self.direction)
        frame = BayFrame(self._bay[0], self._bay[1], self.context.nav.map,
                         heading, 1.0 if self.direction > 0 else -1.0)
        offset = self.setting("parking.stage_along_offset_mm")
        straight = self.setting("parking.straight_mm")
        self._parking = ParkingController(
            frame, bay_mm=self._bay_finder.bay_mm,
            turn_radius_mm=self.pursuit.min_turn_radius_mm,
            line_depth_mm=self.path.map.outer - self.path.half,
            turn_deg=float(self.setting("parking.turn_deg")),
            stage_depth_mm=float(self.setting("parking.stage_depth_mm")),
            stage_along_offset_mm=None if offset is None else float(offset),
            park_depth_mm=float(self.setting("parking.park_depth_mm")),
            end_offset_mm=float(self.setting("parking.end_offset_mm")),
            straight_mm=None if straight is None else float(straight),
            approach_mm=float(self.setting("parking.approach_mm")),
            approach_lookahead_mm=float(self.setting("parking.approach_lookahead_mm")),
            speed=int(self.setting("parking.speed")),
            approach_speed=int(self.setting("parking.approach_speed")))
        print(f"Laps done - parking. Racing line is "
              f"{self.path.map.outer - self.path.half:.0f}mm off the wall, "
              f"turning radius {self.pursuit.min_turn_radius_mm:.0f}mm")
        print(f"  {self._parking.summary()}")

    def parking_caps(self):
        """Step 0: slow down and shorten the lookahead as the bay comes up."""
        if self._parking is None or self._parking.finished:
            return (None, None)
        return self._parking.path_caps()

    def parking_command(self, dt):
        """
        The manoeuvre's steering and speed for this tick, or None to let the
        racing line keep driving - see PathDrivingTask._drive_parking.
        """
        if not self.setting("parking.enabled") or self._bay is None:
            return None
        if self.laps_done < self.laps_goal:
            return None
        if self._parking is None:
            self._start_parking()
        if self._parking.finished:
            return None
        pose = self.context.nav.get_pose()
        # The bay frame's `s` is just a field coordinate, so it means nothing
        # while the robot is somewhere else on the mat - and would happily
        # read as "past the staging point" from the far side of the field.
        if section_of(pose.x, pose.y, self.context.nav.map) != self._bay[0]:
            return None
        return self._parking.update(pose, dt, max_steer=self.pursuit.max_steer_command)

    def _update_pillar_memory(self):
        """
        The pillars the line is steering around, refreshed from the block map.

        Two jobs. It records, once per pillar, how far ahead it was the first
        time it could be steered on at all - the ramp anchor. And it HOLDS a
        pillar across the gaps in its own detection, so that a dropped track
        softens nothing; see Pillar for why that matters more than it sounds.

        On the anchor:

        `approach_mm` on its own is a SCHEDULE: it says the sweep starts when
        the pillar is that far ahead, which only works while the pillar is
        actually on the map that early - and often it is not. A block has to
        be inside MAX_MAPPING_RANGE_MM, inside the camera cone, and seen
        HITS_TO_CONFIRM times before confirmed() will return it at all, and
        after a corner it can be much later than that. A pillar that first
        appears closer than approach_mm lands mid-schedule, and the whole
        offset the schedule says it should already have arrives in one tick -
        the exact step the smoothstep exists to avoid, and the reason raising
        approach_mm past what the camera can see makes the line worse rather
        than smoother.

        Anchoring each pillar's ramp to where it was really first seen turns
        approach_mm from "start exactly here" into "start no earlier than
        here": a late pillar gets a shorter sweep instead of a step, and
        approach_mm can be set to what the line wants without having to also
        be a promise about detection range.

        Measured against `aim_progress` - the lookahead point pure pursuit is
        chasing - because that is the progress the ramp is evaluated at.
        Anchoring off the robot's own progress instead would put the first
        evaluation a whole lookahead into the ramp, which is the step again,
        just smaller.
        """
        now = time.monotonic()
        past = (float(self.setting("blocks.padding_mm"))
                + float(self.setting("blocks.past_mm")))
        hold_s = float(self.setting("blocks.memory_s"))

        # Kept by POSITION, and kept whether or not the pillar is confirmed
        # this instant. A pillar that blinks out for a few frames is the same
        # pillar when it comes back, and keeps both the sweep it had started
        # and the offset it was already steering. Two things forget it:
        # driving past it, which is also what lets it anchor afresh on the
        # next lap, and hold_s of silence, which bounds how long a detection
        # that turned out to be wrong can keep bending the line.
        remembered = [pillar for pillar in self._pillars
                      if self.path.gap(self.aim_progress, pillar.progress) > -past
                      and now - pillar.last_seen <= hold_s]

        for block in self.context.nav.blocks.confirmed():
            if block.color not in SIDE_FOR_COLOR:
                continue
            progress, lateral = self.path.project(block.x, block.y, self.direction)
            index = self._remembered_index(progress, remembered)
            if index is None:
                anchor = self.path.gap(self.aim_progress, progress)
                self._announce_pillar(block.color, anchor)
                remembered.append(Pillar(progress, lateral, block.color, anchor, now))
            else:
                # Position and colour follow the map as it refines them; the
                # anchor never moves, or a pillar re-detected closer would
                # restart its sweep from wherever the robot has got to.
                remembered[index] = remembered[index]._replace(
                    progress=progress, lateral=lateral, color=block.color, last_seen=now)
        self._pillars = remembered
        # The profile is rebuilt from scratch every tick rather than patched,
        # because it is cheap (a handful of nodes) and because a profile that
        # is edited in place is a profile whose continuity depends on the edit
        # being right. Both versions: the one that is driven, and the one the
        # settings asked for, which the debug view draws behind it.
        self._settle_bias()

    # ------------------------------------------------------------------------
    # THE LINE
    # ------------------------------------------------------------------------

    def _settle_bias(self):
        """
        Rebuilds the profile and absorbs however much it moved AT THE POINT THE
        ROBOT IS CHASING, so that a pillar appearing does not yank the pursuit
        target sideways.

        This is what lets _build_line ignore the robot completely. The profile
        is a function of where the pillars are and nothing else - the same
        curve whether the robot is beside it or on the far side of the field -
        which is the only way it can be a stable thing to steer at. What used
        to make that impossible was the pop-in: a pillar confirmed 400mm ahead
        arrives with a profile that says the line should ALREADY be most of the
        way across, and following that literally means the target teleports.

        Two properties matter as much as the smoothing itself, and the first
        version of this had neither:

        LOCAL. The correction is pinned to the point on the lap where the
        change was noticed and faded out over catchup_mm AHEAD of it. It is not
        a shift of the whole profile. A shift of the whole profile means a
        pillar being picked up on one side of the field moves the racing line
        on the other side of it, which is not a smoothing, it is a lap-wide
        offset that has nothing to do with any pillar.

        FINITE. It reaches exactly zero catchup_mm later. Decaying it by a
        fraction each tick instead makes it exponential - never actually zero,
        just small - and since every pillar picked up AND every pillar
        forgotten after being passed re-excites it, an exponential tail never
        gets the chance to run out. That is what made the line take laps to
        settle instead of a metre.

        Only a real change reseeds it. A pillar's mapped position refining by a
        millimetre a tick is already smooth and is passed straight through;
        absorbing that would make the line lag the truth for no gain.
        """
        aim = self.aim_progress
        before = self._offset_at(self._line, aim)
        carried = self._catchup_at(aim)
        # Spent corrections are cleared, not left lying around. `_bias_from` is
        # a point on a LOOP, so a stale value whose window the robot drove out
        # of a lap ago is a value whose window the robot is about to drive into
        # again - gap() wraps, and it would re-apply at full strength.
        if not carried:
            self._bias = 0.0

        self._line = self._build_line(capped=True)
        self._wanted_line = self._build_line(capped=False)

        after = self._offset_at(self._line, aim)
        if float(self.setting("blocks.catchup_mm")) <= 0.0:
            self._bias = 0.0
            return
        if abs(after - before) <= CATCHUP_DEADBAND_MM:
            return          # nothing jumped - let any fade already running run
        # Whatever it takes for the new profile to agree, right here, with what
        # was being driven a moment ago.
        self._bias = before + carried - after
        self._bias_from = aim

    def _catchup_at(self, progress):
        """
        How much of the outstanding correction still applies at `progress`:
        all of it where the change was noticed, none of it catchup_mm further
        on, eased between so the line slides rather than steps.

        Zero everywhere outside that window, which is the point - the lap the
        robot has already driven and the lap beyond the window are the profile
        and nothing else.
        """
        if not self._bias:
            return 0.0
        catchup = float(self.setting("blocks.catchup_mm"))
        if catchup <= 0.0:
            return 0.0
        ahead = self.path.gap(self._bias_from, progress)
        if ahead < 0.0 or ahead >= catchup:
            return 0.0
        lead, settle = self._handles()
        return self._bias * (1.0 - _bezier_ease(ahead / catchup, lead, settle))

    def _build_line(self, capped=True):
        """
        The whole lap's offset profile, as a list of Spans to interpolate
        across.

        The profile is a chain of (progress, offset) nodes joined by cubic
        Beziers, and every node is horizontal - the curve arrives and leaves
        parallel to the racing line - so the whole lap is smooth by
        CONSTRUCTION. Nothing here has to be checked for continuity afterwards,
        which is the point: the previous profile built each pillar its own
        curve and then added them up, and the joins between them were where
        every artefact came from.

        Each pillar lays down two nodes at its own offset, a flat part either
        side of it that actually does the passing. Between one pillar's flat
        part and the next there is either room to come back to the racing line,
        in which case two more nodes at zero are laid down and the line eases
        out and back in, or there is not, in which case the two flats are
        joined directly and the line crosses straight from one dodge to the
        other. That second case is the handover, and it needs no special
        handling at all - it is one more Bezier between two nodes.

        I/O:
            capped: False builds the profile the settings ASK for, ignoring
                    where each pillar was actually first seen and what the
                    corridor will take. Drawn dim on the debug view next to the
                    real one so the difference is visible.
            return: list of Span, in increasing progress, covering the lap
        """
        pillars = self._spaced_out(
            sorted(self._pillars, key=lambda pillar: pillar.progress))
        if not pillars:
            return []

        offsets = [self._pillar_offset_mm(pillar, capped) for pillar in pillars]
        spacings = [self.path.length if len(pillars) == 1 else
                    self.path.gap(pillar.progress,
                                  pillars[(index + 1) % len(pillars)].progress)
                    % self.path.length
                    for index, pillar in enumerate(pillars)]

        # Every gap is divided BEFORE any node is placed. Doing it pillar by
        # pillar instead lets a pillar's approach-side flat run back past the
        # one behind it, which puts the nodes out of order along the lap - and
        # a profile whose nodes are out of order is not a curve, it is a jump.
        shares = [self._share_gap(offsets[index],
                                  offsets[(index + 1) % len(pillars)],
                                  spacings[index],
                                  pillars[(index + 1) % len(pillars)], capped)
                  for index in range(len(pillars))]

        nodes = []
        for index, pillar in enumerate(pillars):
            following = (index + 1) % len(pillars)
            after, _, room = shares[index]
            _, before, _ = shares[index - 1]
            nodes.extend(self._nodes_between(
                pillar, pillars[following], offsets[index], before, after, room,
                spacings[index], capped))
        return self._spans_from(nodes)

    def _spaced_out(self, pillars):
        """
        The pillars with any that sit on top of one another dropped, keeping
        whichever of a pair was seen most recently.

        _update_pillar_memory already merges sightings within
        ANCHOR_SAME_PILLAR_MM into one pillar, so this should never have
        anything to do. It is here because the cost of being wrong about that
        is not a slightly worse line: two pillars a few mm apart wanting
        opposite sides ask the profile to cross the whole corridor in those few
        mm, which is not a steep curve, it is a JUMP - the one thing this
        profile is built not to contain. A cheap guard against a state that
        should be impossible is worth more than the assumption.
        """
        kept = []
        for pillar in pillars:
            if kept and abs(self.path.gap(
                    kept[-1].progress, pillar.progress)) < ANCHOR_SAME_PILLAR_MM:
                if pillar.last_seen > kept[-1].last_seen:
                    kept[-1] = pillar
                continue
            kept.append(pillar)
        # The loop closes, so the last one can crowd the first.
        while len(kept) > 1 and abs(self.path.gap(
                kept[-1].progress, kept[0].progress)) < ANCHOR_SAME_PILLAR_MM:
            kept.pop() if kept[0].last_seen >= kept[-1].last_seen else kept.pop(0)
        return kept

    def _nodes_between(self, pillar, following, offset, before, after, room,
                       spacing, capped):
        """
        The nodes from one pillar's flat part to the start of the next one's:
        the flat itself, and whatever joins it to its neighbour.

        `before` and `after` come from the two gaps either side of this pillar,
        already divided - see _build_line - so the flat part placed here cannot
        run into its neighbours' whatever the pillars are doing.
        """
        nodes = [Node(pillar.progress - before, offset),
                 Node(pillar.progress + after, offset)]

        leaving = min(float(self.setting("blocks.past_mm")), room)
        arriving = min(float(self.setting("blocks.approach_mm")), room - leaving)
        if room > leaving + arriving:
            # Room to spare: come back to the racing line in between, rather
            # than carrying a dodge along a stretch of lap that does not need
            # one. The two nodes at zero are what hold it there.
            nodes.append(Node(pillar.progress + after + leaving, 0.0))
            nodes.append(Node(pillar.progress + after + room - arriving, 0.0))
        return nodes

    def _share_gap(self, offset, next_offset, spacing, following, capped):
        """
        How the gap between two pillars is divided: flat part for the one being
        left, flat part for the one being reached, and the curve between them.

        The CURVE is served first, which is the whole change from the previous
        version. Two pillars on opposite sides 700mm apart need the line to
        cross 610mm in that 700mm whatever anyone would prefer, and taking
        padding_mm off each end first leaves 200mm to do it in - a line at 70
        degrees to the racing line, which pure pursuit does not follow, it cuts
        across. Sizing the curve for max_lean_deg first and giving the flats
        what is left over spends the gap on the part that has to happen.

        When even the whole gap is not enough for that lean - pillars closer
        together than the dodge is wide - the flats keep a token share and the
        curve takes the rest and leans as hard as it must. Steep beats absent:
        the robot has to get round the pillar.

        I/O:
            return: (flat after this pillar, flat before the next, curve length)
        """
        lead, settle = self._handles()
        rise = 1.0 - (lead + settle) / 2.0
        lean = math.tan(math.radians(clamp(
            float(self.setting("blocks.max_lean_deg")), 5.0, 85.0)))
        wanted_curve = abs(next_offset - offset) / max(rise * lean, 1e-6)

        padding = float(self.setting("blocks.padding_mm"))
        floor = spacing * MIN_PADDING_SHARE
        share = clamp((spacing - wanted_curve) / 2.0, floor, padding)
        after = min(share, padding)
        before_next = min(share, self._entry_padding_mm())
        return after, before_next, spacing - after - before_next

    def _spans_from(self, nodes):
        """
        Nodes to Spans, in order, closing the loop.

        Nodes can arrive out of order or on top of one another when pillars are
        packed tightly - a flat part that wanted to start before the previous
        one ended. Rather than reject that, the profile is walked forwards and
        anything that would step backwards is pulled up to where the walk has
        got to, which turns an impossible overlap into a zero-length span.
        """
        spans, walked = [], nodes[0].progress
        for index, node in enumerate(nodes):
            following = nodes[(index + 1) % len(nodes)]
            start = max(walked, node.progress)
            finish = (nodes[0].progress + self.path.length
                      if index == len(nodes) - 1 else following.progress)
            spans.append(Span(start, max(0.0, finish - start),
                              node.offset, following.offset))
            walked = start
        return spans

    def _pillar_offset_mm(self, pillar, capped):
        """
        Where the line has to sit beside this pillar: its own lateral position
        plus the room the robot needs to get past it on the right side.

        Capped by the tightest the corridor gets ANYWHERE the dodge reaches,
        not just beside the pillar - see _tightest_bounds_mm. Capping the
        offset the curve is built from, rather than clipping the curve
        afterwards, is what keeps it a clean Bezier: a clip flattens the top
        into a plateau and puts a corner at each end of it, which is the
        artefact this profile exists to avoid.

        The usual case is a corner. A line held `d` mm inside an arc of radius
        R is itself an arc of radius R - d, so an inward dodge spends the
        corner radius one for one - at d = R the offset line collapses to a
        point and past that it turns inside out.
        """
        offset = (pillar.lateral + SIDE_FOR_COLOR[pillar.color]
                  * self._required_offset_mm(pillar.color))
        if not capped:
            return offset
        low, high = self._tightest_bounds_mm(pillar)
        return clamp(offset, low, high)

    def _tightest_bounds_mm(self, pillar):
        """
        The biggest offset this pillar's dodge can take without any part of it
        leaving the corridor - flat part and transitions alike.

        The obvious version of this measures the narrowest the corridor gets
        anywhere the dodge reaches and caps the offset at that. It is wrong in
        both directions at once. Measured over the whole dodge it is far too
        harsh: the transitions reach 1.6m either side of the pillar, so a
        pillar anywhere within 1.6m of the parking bay's guarded section had
        its whole pass cut to what the guard allows, which on this geometry
        halved the clearance at pillars nowhere near the bay. Measured over the
        flat part alone it is too lax the other way: the transition then runs
        into the tight stretch carrying an offset the corridor will not take,
        and the pointwise clamp flattens it - a 151mm slice out of the ramp
        with a 74-degree corner at each end of the flat spot.

        Both come from asking the wrong question. The transitions do not need
        the full width - they need whatever the profile is actually AT that
        point. A ramp passing a tight spot at 30% of its offset needs 30% of
        the room, so the offset it permits is the room divided by 30%. Taking
        the smallest of those round the whole dodge gives the largest offset
        that fits everywhere, with nothing left for the clamp to remove.
        """
        padding = float(self.setting("blocks.padding_mm"))
        approach = float(self.setting("blocks.approach_mm"))
        past = float(self.setting("blocks.past_mm"))
        lead, settle = self._handles()

        low, high = -float("inf"), float("inf")
        gap = -(padding + past)
        while gap <= padding + approach:
            here = gap
            gap += BOUNDS_SAMPLE_MM
            # How much of the offset the profile carries this far from the
            # pillar - 1 across the flat part, easing to 0 at the ends.
            if abs(here) <= padding:
                fraction = 1.0
            elif here > 0.0:
                fraction = 1.0 - _bezier_ease((here - padding) / approach, lead, settle)
            else:
                fraction = 1.0 - _bezier_ease((-here - padding) / past, lead, settle)
            if fraction <= 1e-3:
                continue
            here_low, here_high = self._lateral_bounds_mm(pillar.progress - here)
            low = max(low, here_low / fraction)
            high = min(high, here_high / fraction)
        return low, high

    def _entry_padding_mm(self):
        """The flat part on a pillar's approach side."""
        return float(self.setting("blocks.padding_mm"))

    def _handles(self):
        """The two Bezier handle positions, clamped to keep the curve
        monotonic along the track."""
        lead = clamp(float(self.setting("blocks.bezier_lead")), 0.02, 0.9)
        settle = clamp(float(self.setting("blocks.bezier_settle")), 0.02, 0.9)
        if lead + settle > 1.0:
            scale = 1.0 / (lead + settle)
            lead, settle = lead * scale, settle * scale
        return lead, settle

    def _steerable_radius_mm(self):
        """
        The tightest arc a dodge is allowed to put in the line: the robot's own
        turning circle with turn_radius_margin kept in hand, because a line it
        can only just hold leaves nothing to correct tracking error with.
        """
        return (self.pursuit.min_turn_radius_mm
                * float(self.setting("blocks.turn_radius_margin")))

    def _corner_room_mm(self):
        """
        How far the line may be shifted toward the inside of a bend before the
        shifted line is tighter than the robot can steer. Every corner on this
        loop is the same arc, so one of them answers for all four.
        """
        spans = self.path.bend_spans(self.direction)
        if not spans:
            return float("inf")
        return self.path.inward_limit_mm(
            (spans[0][0] + spans[0][1]) / 2.0, self.direction,
            self._steerable_radius_mm())

    def _announce_pillar(self, color, anchor_mm):
        """
        Says how far out a pillar was when it was first confirmed, the moment
        that is decided, and says so LOUDLY when that is too late to take up
        the dodge in time.

        The dodge itself no longer changes shape for a late pillar - the
        profile is a function of where the pillars are and nothing else, so
        every pillar gets the same curve (see _build_line). What being seen
        late costs is the DISTANCE to slide onto that curve: the line starts
        wherever it was and closes the gap over catchup_mm, so a pillar
        confirmed 400mm out is passed part-way across rather than fully across.

        That makes this the number to watch when a pass looks shallow, and it
        is one that cannot be tuned into existence - approach_mm cannot help a
        pillar the camera has not found yet.
        """
        catchup = float(self.setting("blocks.catchup_mm"))
        if anchor_mm >= catchup:
            print(f"{color.value} pillar first seen {anchor_mm:.0f}mm out - "
                  f"room to take the dodge up in full")
            return
        if anchor_mm <= -float(self.setting("blocks.padding_mm")):
            # Behind us in LAP terms, which is routine rather than alarming:
            # the camera sees clean across the field, so most of what it picks
            # up on a corner belongs to a stretch of track already driven. It
            # does not need dodging from here - the lap comes back round to it.
            return
        reached = clamp(anchor_mm / catchup, 0.0, 1.0) * 100.0
        print(f"WARNING: {color.value} pillar first seen only {anchor_mm:.0f}mm out, so "
              f"the line reaches about {reached:.0f}% of its dodge before passing it "
              f"(catchup_mm is {catchup:.0f}). Raising approach_mm cannot help this "
              f"pillar - raising blocks.map_range_mm "
              f"({self.context.nav.blocks.max_range_mm:.0f}mm), or fixing why it was "
              f"seen late, is what would.")

    def _remembered_index(self, progress, pillars):
        """
        Where in `pillars` the pillar at that progress is remembered, or None.

        Matched by distance along the lap rather than by identity, because
        identity is not stable - see ANCHOR_SAME_PILLAR_MM.
        """
        for index, pillar in enumerate(pillars):
            if abs(self.path.gap(pillar.progress, progress)) <= ANCHOR_SAME_PILLAR_MM:
                return index
        return None

    def _required_offset_mm(self, color):
        """
        How far from the PILLAR'S OWN position the robot's steered centerline
        must sit for its body to actually clear it by clearance_mm.

        Three parts, all real mm: clearance_mm is the gap you want between
        the robot's body and the pillar's face; robot_half_width_mm converts
        that from a body-relative gap into a centerline-relative one, since
        pure pursuit steers the centerline, not the edge; BLOCK_SIZE_MM/2
        reaches from the pillar's own center out to its near face.

        Nothing here depends on the colour except the optional per-colour pad,
        which is 0 by default and meant to stay there: red and green are the
        same block and the robot is the same width going past either, so the
        dodge is the same SIZE for both and only its direction differs (see
        SIDE_FOR_COLOR). A non-zero pad is for living with a detection that
        mis-ranges one colour, not for being more careful on one side - and it
        costs real clearance, because the wider offset is the first thing the
        wall clamp takes back. See blocks.red_extra_clearance_mm.
        """
        extra_key = {Color.RED: "blocks.red_extra_clearance_mm",
                     Color.GREEN: "blocks.green_extra_clearance_mm"}[color]
        return (float(self.setting("blocks.clearance_mm"))
               + float(self.setting("blocks.robot_half_width_mm"))
               + float(self.setting(extra_key))
               + BLOCK_SIZE_MM / 2.0)

    def _wall_room_mm(self):
        """
        How far the steered centerline may move each way before the robot's
        BODY would be closer than wall_clearance_mm to the outer wall or the
        centre block.

        Returned as two numbers rather than one, because the corridor is not
        symmetric about the racing line unless wall_margin_mm centres it - and
        collapsing them to the smaller gives up real room on the wider side for
        no reason. On the current geometry that was costing every dodge toward
        the centre block 52mm it could have had.

        Same body-vs-centerline correction either way: without adding
        robot_half_width_mm back in, this caps the centerline's distance from
        the wall, not the body's, and the body ends up wall_clearance_mm short
        of the real wall by however wide the chassis is.

        I/O:
            return: (toward_wall_mm, toward_block_mm)
        """
        wall = (float(self.setting("blocks.wall_clearance_mm"))
                + float(self.setting("blocks.robot_half_width_mm")))
        return self.path.lateral_room_mm(wall)

    def _wall_limit_mm(self):
        """The tighter of the two sides - what a dodge can count on either
        way, and what the startup fit check is measured against."""
        return min(self._wall_room_mm())

    def _bay_wall_limit_mm(self, progress):
        """
        How far the line may move TOWARD THE OUTER WALL near the parking bay,
        or None where the bay does not constrain it.

        The bay walls stick 200mm out from the outer wall. `wall_clearance_mm`
        assumes a flat wall, so on the current numbers it lets the body reach
        80mm from it - straight through a bay wall. This is the fix.

        It applies AT THE BAY and nowhere else, which is worth stating plainly
        because it did not always: it used to cover the whole starting section
        until BayFinder had located the bay, on the reasoning that the first
        pass necessarily happens before anything knows where it is. That is a
        quarter of the lap held 160mm off the wall on every lap, and on this
        geometry it cut a pillar dodge in that section to roughly half the
        clearance it asked for.

        What makes the narrow version safe is a fact about the field rather
        than anything the robot measures: the competition never puts a pillar
        within PILLAR_FREE_MM of a parking space. So wherever the bay turns out
        to be, no pillar's pass is beside it, and the line is on or near the
        racing line there - which is already 274mm clear of a bay wall. The
        guard is there for the case the geometry does not cover, not for the
        common one.

        `parking.guard_before_found` puts the old behaviour back for a mat
        where that guarantee does not hold - a practice layout with a pillar
        parked next to the bay, say. Once the bay HAS been found the window
        around it is always guarded, because at that point its position is
        known and there is nothing to trade off.

        Nothing here reads the lidar or estimates where any wall is. The outer
        wall is exact, known field geometry and `section_of` is a lookup
        against the predefined map; the only thing ever uncertain is where
        along that wall the bay sits, and BayFinder answers that.
        """
        if not self.setting("parking.enabled") or self._start_section is None:
            return None

        if self._bay_progress is not None:
            # Known: guard a window around it. PILLAR_FREE_MM is why this
            # costs nothing - no pillar's pass is inside that window either.
            window = float(self.setting("parking.window_mm"))
            if abs(self.path.gap(progress, self._bay_progress)) > window:
                return None
        else:
            if not self.setting("parking.guard_before_found"):
                return None
            x, y, _ = self.path.pose_at(progress, self.direction)
            if section_of(x, y, self.context.nav.map) != self._start_section:
                return None

        clearance = (WALL_LENGTH_MM
                     + float(self.setting("parking.wall_margin_mm"))
                     + float(self.setting("blocks.robot_half_width_mm")))
        # Only the wall side - deliberately NOT path.lateral_limit, which
        # takes the min with the centre-block side and would tighten the wrong
        # direction as a side effect.
        return max(0.0, self.path.map.outer - self.path.half - clearance)

    def _lateral_bounds_mm(self, progress):
        """
        The (low, high) the line may be shifted between at `progress`, in the
        travel frame where + is to the right.

        Asymmetric near the bay: which SIGN points at the outer wall depends
        on which way round the loop is being driven, so a symmetric clamp
        would restrict the centre-block side by mistake half the time. Same
        goes for the inside of a bend, for the same reason.

        The corner term here is a BACKSTOP, not the mechanism: _pillar_offset_mm
        has already capped every dodge to fit the corner it sits in, so in
        normal running this never binds. What it catches is the line arriving
        somewhere the sizing did not account for - a pillar's position moving
        under it, or two dodges blending - and it is worth catching, because
        the failure it prevents is the offset line turning inside out rather
        than merely running wide.
        """
        toward_wall, toward_block = self._wall_room_mm()
        # Which SIGN points at the outer wall depends on which way round the
        # loop is being driven: lateral is positive to the right of travel, and
        # the wall is on the right going the +1 way round.
        if self.direction > 0:
            low, high = -toward_block, toward_wall
        else:
            low, high = -toward_wall, toward_block

        inward = self.path.inward_limit_mm(
            progress, self.direction, self._steerable_radius_mm())
        if self.path.inward_sign(self.direction) < 0.0:
            low = max(low, -inward)
        else:
            high = min(high, inward)

        bay = self._bay_wall_limit_mm(progress)
        if bay is None:
            return low, high
        if self.direction > 0:
            return low, min(high, bay)
        return max(low, -bay), high

    def target_lateral_mm(self, progress, capped=True):
        """
        Where the racing line should sit at `progress`, to pass the pillars
        correctly.

        A lookup into the profile built by _build_line, eased across the span
        it lands in. Because that profile is a chain of Beziers that are
        horizontal at every join, there is nothing to blend and nothing that
        can step: the answer at any point on the lap comes from exactly one
        curve, and the curve either side of it agrees with it in both value and
        slope.

        Clamped by _lateral_bounds_mm at the end, which stops the line pushing
        the robot's body closer than wall_clearance_mm to the wall or the
        centre block, and stops an inward dodge tightening a corner past what
        the steering can hold. In normal running neither binds -
        _pillar_offset_mm has already capped the offset the profile was built
        from - so this is a backstop against a pillar's mapped position moving
        under a profile that is already built.
        """
        spans = self._line if capped else self._wanted_line
        if not spans and not self._bias:
            return 0.0


        # `_bias` only ever applies to the line being driven. The dim overlay
        # line is what the settings ASK for, and what they ask for does not
        # depend on when a pillar happened to be spotted.
        offset = self._offset_at(spans, progress)
        if capped:
            offset += self._catchup_at(progress)
        low, high = self._lateral_bounds_mm(progress)
        return clamp(offset, low, high)

    def _offset_at(self, spans, progress):
        """
        The profile's own value at `progress`, before any bias or clamp.

        An empty profile is the racing line, which is also what a lap with no
        pillars on it should read - and _settle_bias asks for it in exactly
        that state, on the tick a pillar first appears.
        """
        if not spans:
            return 0.0
        lead, settle = self._handles()
        distance = (progress - spans[0].start) % self.path.length
        for span in spans:
            reach = span.start - spans[0].start + span.length
            if distance > reach:
                continue
            if span.length <= 0.0:
                return span.end
            fraction = (distance - (span.start - spans[0].start)) / span.length
            if span.begin == span.end:
                return span.begin
            return span.begin + (span.end - span.begin) * _bezier_ease(
                fraction, lead, settle)
        return spans[-1].end

    def _nearest_pillar(self, progress):
        """The pillar whose stretch of lap `progress` is in, or None."""
        best = None
        for pillar in self._pillars:
            gap = abs(self.path.gap(progress, pillar.progress))
            if best is None or gap < abs(self.path.gap(progress, best.progress)):
                best = pillar
        return best

    def is_finished(self):
        """
        The round is over when the robot is PARKED, not when the laps are.

        With parking on, running out of laps is not a reason to stop - it is
        the cue to start looking for somewhere to stop. If the bay is never
        found the robot keeps lapping and the runner's own time limit ends the
        round, which scores the laps rather than throwing them away.
        """
        if self._stop_reason:
            print(f"Stopping: {self._stop_reason}")
            return True
        if not self.setting("parking.enabled"):
            return self.laps_done >= self.laps_goal
        if self._parking is not None:
            return self._parking.finished
        # Laps done, no bay found yet: keep going round and keep looking. Not
        # forever, though - without this bound a round with no bay in front of
        # it (a bench test, a practice mat without the walls) never ends at
        # all and just burns the clock.
        return self.laps_done >= self.laps_goal + float(self.setting("parking.extra_laps"))

    def _dodge_status(self):
        """
        What the line is doing right now, as opposed to what the camera has
        found.

        The block counts next door answer "did we SEE it". This answers "is the
        line acting on it", which is the first thing to check when a dodge does
        not appear on the mat and the second thing to check when it appears in
        the wrong place. It says what the line is at, what the nearest pillar
        asked for, and - when those differ - which limit took the difference.
        """
        offset = self.target_lateral_mm(self.aim_progress)
        pillar = self._nearest_pillar(self.aim_progress)
        if pillar is None:
            return f"dodge {offset / 10:+5.1f}cm (no pillar in range)"

        gap = self.path.gap(self.aim_progress, pillar.progress)
        wanted = (pillar.lateral + SIDE_FOR_COLOR[pillar.color]
                  * self._required_offset_mm(pillar.color))
        driven = self._pillar_offset_mm(pillar, capped=True)
        return (f"dodge {offset / 10:+5.1f}cm (next {pillar.color.value} "
                f"{gap / 10:+.0f}cm away, wants the line at {wanted / 10:+.0f}cm, "
                f"passing at {driven / 10:+.0f}cm"
                f"{self._short_by(pillar, wanted, driven)}"
                f"{self._catching_up()}{self._held_for(pillar)})")

    def _catching_up(self):
        """
        How far the line still is from the profile it is heading for, or ""
        once it is on it.

        Non-zero means a pillar turned up late enough that the line is still
        sliding across - see _settle_bias. Worth saying out loud because it
        looks exactly like a shallow dodge from outside, and the fix is at the
        other end of the pipeline: the camera found the pillar too late.
        """
        outstanding = abs(self._catchup_at(self.aim_progress))
        if outstanding < 1.0:
            return ""
        return f", still {outstanding / 10:.0f}cm from the line"

    def _short_by(self, pillar, wanted, driven):
        """
        Why a pillar is being passed closer than clearance_mm asked for, or ""
        when it is not.

        Two different faults look identical on the mat and need opposite fixes:
        the corridor running out sideways (lower blocks.clearance_mm, or move
        the racing line) against the corner arc being too tight to bend the
        line into (lower path.wall_margin_mm, which widens the arcs). Naming
        which one it is here saves guessing at it between runs.
        """
        if abs(wanted - driven) <= 0.5:
            return ""
        corner = self._corner_room_mm()
        inward = driven * self.path.inward_sign(self.direction)
        bay = self._bay_wall_limit_mm(pillar.progress)
        if bay is not None and abs(driven) >= bay - 0.5 and inward < corner - 0.5:
            blame = "bay-guard"
        elif inward >= corner - 0.5:
            blame = "corner"
        else:
            blame = "wall"
        gap = (abs(driven) - float(self.setting("blocks.robot_half_width_mm"))
               - BLOCK_SIZE_MM / 2.0)
        return f" - {blame}-limited, {gap / 10:.0f}cm of body gap not {self.setting('blocks.clearance_mm') / 10:.0f}"

    def _held_for(self, pillar):
        """
        How long this pillar has been steering the line on memory alone, or ""
        while the camera can still see it. A dodge that is being held is not a
        fault - it is the point - but a dodge held for most of its approach
        says the detection is dropping out, not that the ramp is wrong.
        """
        stale = time.monotonic() - pillar.last_seen
        return f" HELD {stale:.1f}s" if stale > 0.15 else ""

    def status(self):
        line = (f"{super().status()}  {self.context.nav.blocks.summary()}"
                f"  {self._dodge_status()}")
        if self.context.vision is not None:
            line = f"{line}  {self.context.vision.status_line()}"
        return line

    def _draw_overlay(self, canvas, to_px):
        """
        The plain racing line, the bent one the robot is actually on, and -
        dimmer, behind it - the one the config asked for.

        The two bent lines are the same wherever the settings are getting what
        they want. Where they differ, the dim line is what padding_mm,
        approach_mm and past_mm describe and the bright one is what the pillar
        actually left room for, which is the difference between a knob that
        does nothing and a knob that is being overruled. Nothing else on this
        view distinguishes those two, and they need very different fixes.
        """
        super()._draw_overlay(canvas, to_px)

        wanted, actual = [], []
        for step in range(0, int(self.path.length), 40):
            progress = self.progress + step
            for points, capped in ((wanted, False), (actual, True)):
                x, y = self.path.point_at(
                    progress, self.direction,
                    self.target_lateral_mm(progress, capped=capped))
                points.append(to_px(x, y))
        if wanted != actual:
            cv2.polylines(canvas, [np.array(wanted, dtype=np.int32)],
                          isClosed=True, color=(70, 55, 25), thickness=1)
        if actual:
            cv2.polylines(canvas, [np.array(actual, dtype=np.int32)],
                          isClosed=True, color=(180, 140, 60), thickness=1)

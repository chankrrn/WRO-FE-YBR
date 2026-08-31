#!/usr/bin/env python3
"""
Interactive field view for both rounds: change the config, see what it does to
the path the robot drives.

    uv run python test_plan_view.py              opens http://127.0.0.1:8770
    uv run python test_plan_view.py --port 9000
    uv run python test_plan_view.py --no-browser
    uv run python test_plan_view.py --selftest   geometry and config checks

WHAT IT IS FOR. Every number in tasks/*/config.toml changes the line the robot
takes, and most of them do so in a way that is impossible to picture from the
file. This puts the two side by side: the config as it is on disk (BASELINE)
against the config with your edits (VARIANT), drawn on the same field, and -
when you ask for it - both of them actually driven.

Three things it will answer:

    THE LINE          path.wall_margin_mm and path.corner_radius_mm decide the
                      racing line itself. Both lines are drawn together, live,
                      as you type. No simulation needed.

    THE PLAN          (final round) where the goal planner puts the robot to
                      get past the pillars you place, whether it can give them
                      the clearance blocks.clearance_mm asks for, and which
                      side it really passes on.

    THE DRIVING       both configs run through the REAL round - the real
                      QualificationTask or FinalTask, the real pure pursuit,
                      the real particle filter, against test_driving.py's
                      simulated robot with its servo lag and slew limit. Laps,
                      time, how far off its own line it drifted, how close it
                      came to a wall.

NOTHING HERE RE-IMPLEMENTS ANY DRIVING. The browser is a drawing surface and an
input device. The lines come from classes/racing_line.py, the plans from
classes/goal_planner.py, and the driven traces from test_driving.simulate()
running the round's own task class. A second implementation of any of it in
JavaScript would agree with itself beautifully and tell you nothing about the
robot.

WHAT THE SIMULATOR IS NOT. It holds the truth and the config holds a belief
about it, and the two are deliberately allowed to disagree - the sim's chassis
is 240x120mm whatever blocks.robot_half_width_mm says, and its full-lock angle
is whatever `steer gain error` is set to whatever pursuit.max_road_wheel_deg
says. The Simulator panel is where you make them disagree on purpose. That is
the point: a run that quietly assumed the config was right could not reproduce
a single failure that actually happens on the mat.

REQUIREMENTS. `uv sync` gets everything, on the Pi and on a dev machine alike.
The line and plan views need only numpy; DRIVING additionally needs the
packages the round itself imports (opencv-python for the camera pipeline,
pyserial for the motor link), and the page says so rather than breaking if
they are absent - the rest of the tool keeps working without them.

RPi.GPIO is marked linux-only in pyproject.toml because it is a C extension
against the Pi's own headers and cannot build anywhere else. Nothing in the
simulators touches GPIO, so off the Pi its absence costs nothing.
"""
import argparse
import contextlib
import io
import itertools
import json
import math
import queue
import sys
import threading
import time
import traceback
import types
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from classes.field_map import FieldMap
from classes.goal_planner import BLOCK_RADIUS_MM, GoalPlanner, Obstacle
from classes.parking import (NOMINAL_BAY_MM, WALL_LENGTH_MM,
                             WALL_THICKNESS_MM, bay_pose, wall_rects)
from classes.pure_pursuit import PurePursuit
from classes.racing_line import RacingLine
from utils.task_config import TaskConfig

HERE = Path(__file__).resolve().parent
PAGE_PATH = HERE / "test_plan_view.html"

# The pillar's real 50mm square, recovered from the circumscribed radius the
# planner collides against. Taken this way round rather than imported from
# block_map, which pulls in cv2 - the two are the same number by construction
# (BLOCK_RADIUS_MM is BLOCK_SIZE_MM * sqrt(2) / 2) and the one that matters
# for a verdict on the screen is the radius, which comes from the planner.
BLOCK_SIZE_MM = BLOCK_RADIUS_MM * math.sqrt(2.0)

# GREEN is passed on the block's left, RED on its right - the same mapping
# FinalTask.SIDE_FOR_COLOR uses, in the string form the browser talks in.
SIDE_FOR_COLOR = {"red": +1.0, "green": -1.0}

# ============================================================================
# The rounds
# ============================================================================
# Task classes are named rather than imported: importing FinalTask reaches
# cv2, and the line and plan views are useful on a machine that has not got it.
ROUNDS = {
    "qualification": {
        "config": HERE / "tasks" / "qualification" / "config.toml",
        "task": ("tasks.qualification.task", "QualificationTask"),
        "pillars": False,
        "label": "Qualification - three laps of the empty mat",
    },
    "final": {
        "config": HERE / "tasks" / "final" / "config.toml",
        "task": ("tasks.final.task", "FinalTask"),
        "pillars": True,
        "label": "Final - three laps past red/green pillars, then park",
    },
}
DEFAULT_ROUND = "final"

# Knobs the panel offers, in the order they are shown. Grouped rather than
# alphabetical because tuning happens by subject: what shape is the line, how
# hard does the follower chase it, how much room does a pass get.
COMMON_GROUPS = [
    ("The racing line", ["path.wall_margin_mm", "path.corner_radius_mm",
                         "path.resolution_mm"]),
    ("Lookahead", ["pursuit.lookahead_base_mm", "pursuit.lookahead_per_speed_mm",
                   "pursuit.lookahead_min_mm", "pursuit.lookahead_max_mm",
                   "pursuit.corner_lookahead_fraction"]),
    ("Steering", ["pursuit.max_road_wheel_deg", "pursuit.max_steer_command",
                  "pursuit.wheelbase_mm", "pursuit.rear_axle_offset_mm",
                  "pursuit.lag_compensation_s", "steer.max_rate_units_s"]),
    ("Speed", ["speed.base", "speed.corner", "speed.lost", "speed.minimum",
               "laps.goal"]),
]
FINAL_GROUPS = [
    ("Pass geometry", ["blocks.clearance_mm", "goals.min_clearance_mm",
                       "blocks.wall_clearance_mm", "blocks.turn_radius_margin",
                       "speed.compromised"]),
    ("Robot body", ["blocks.robot_half_width_mm", "goals.robot_front_mm",
                    "goals.robot_rear_mm"]),
    ("Plan shape", ["goals.horizon_mm", "goals.route_spacing_mm",
                    "goals.approach_mm", "goals.exit_mm", "goals.max_gates",
                    "goals.replan_interval_s", "goals.replan_cross_track_mm"]),
]

# Knobs where an EMPTY BOX IS A REAL SETTING rather than a mistake: unset means
# "work it out", and that is a different line from any number you could type.
# Anywhere else an empty box falls back to the file, because a cleared speed is
# a slip and driving at None is not a thing anyone wants to watch happen.
NULLABLE = {"path.wall_margin_mm", "path.corner_radius_mm",
            "steer.max_rate_units_s", "blocks.map_range_mm"}

# What the SIMULATOR is, as opposed to what the config BELIEVES. These are the
# knobs that let the two disagree, which is the only way to find out which of
# them a weave is actually coming from - see the tuning notes at the top of
# tasks/qualification/config.toml.
SIM_KNOBS = {
    "servo_lag_s": 0.12,          # first-order response of the wheels
    "servo_rate_deg_s": 400.0,    # servo slew limit, command units/s
    "steer_gain_error": 1.0,      # true full lock / what the config claims
}

# path_task's DEFAULTS, duplicated for the case where it cannot be imported
# (it reaches cv2 through DebugView). Only the keys the panel and the planner
# need; defaults() prefers the real thing whenever it is importable, so this
# is a fallback and not a second source of truth.
FALLBACK_DEFAULTS = {
    "laps.goal": 3,
    "speed.base": 70, "speed.corner": 60, "speed.lost": 50,
    "speed.minimum": 40, "speed.compromised": 45,
    "path.wall_margin_mm": None, "path.corner_radius_mm": None,
    "path.resolution_mm": 20.0,
    "pursuit.wheelbase_mm": 165.0, "pursuit.lookahead_base_mm": 260.0,
    "pursuit.lookahead_per_speed_mm": 3.0, "pursuit.lookahead_min_mm": 250.0,
    "pursuit.lookahead_max_mm": 700.0, "pursuit.max_road_wheel_deg": 30.0,
    "pursuit.max_steer_command": 80.0, "pursuit.rear_axle_offset_mm": 0.0,
    "pursuit.corner_lookahead_fraction": 0.8, "pursuit.lag_compensation_s": 0.0,
    "steer.max_rate_units_s": None,
    "blocks.clearance_mm": 150.0, "blocks.robot_half_width_mm": 100.0,
    "blocks.wall_clearance_mm": 80.0, "blocks.turn_radius_margin": 1.15,
    "blocks.map_range_mm": None,
    "goals.min_clearance_mm": 45.0, "goals.horizon_mm": 2200.0,
    "goals.route_spacing_mm": 550.0, "goals.approach_mm": 450.0,
    "goals.exit_mm": 350.0, "goals.max_gates": 3,
    "goals.robot_front_mm": 200.0, "goals.robot_rear_mm": 40.0,
    "goals.replan_interval_s": 0.1, "goals.replan_cross_track_mm": 120.0,
    "parking.enabled": True,
}

_DEFAULTS_CACHE = {}


def defaults():
    """path_task's DEFAULTS where importable, the fallback table otherwise."""
    if "value" not in _DEFAULTS_CACHE:
        try:
            from tasks.path_task import DEFAULTS
            merged = dict(FALLBACK_DEFAULTS)
            merged.update(DEFAULTS)
            _DEFAULTS_CACHE["value"] = merged
        except Exception:                               # noqa: BLE001
            _DEFAULTS_CACHE["value"] = dict(FALLBACK_DEFAULTS)
    return _DEFAULTS_CACHE["value"]


def panel_groups(round_name):
    groups = list(COMMON_GROUPS)
    if ROUNDS[round_name]["pillars"]:
        groups = groups + FINAL_GROUPS
    return groups


def panel_keys(round_name):
    return [key for _, keys in panel_groups(round_name) for key in keys]


def read_config(round_name):
    """
    This round's tunables as the panel shows them: the TOML on disk with
    path_task's defaults underneath.
    """
    path = ROUNDS[round_name]["config"]
    config = TaskConfig.load(path) if path.exists() else None
    table = defaults()
    settings = {}
    for key in panel_keys(round_name):
        fallback = table.get(key)
        settings[key] = fallback if config is None else config.get(key, fallback)
    return settings


class Pose:
    """Just enough of NavigationManager's Pose for the planner and pursuit."""

    __slots__ = ("x", "y", "heading")

    def __init__(self, x, y, heading):
        self.x = float(x)
        self.y = float(y)
        self.heading = float(heading) % 360.0


# ============================================================================
# The parking bay
# ============================================================================
# Which way a bay in each section OPENS - out of the outer wall and into the
# field, degrees clockwise from +Y. parking.wall_heading_of gives the other
# one, along the wall, which is what the robot is parked facing.
BAY_OPENING_HEADING = {"south": 0.0, "east": 270.0, "north": 180.0, "west": 90.0}


def bay_walls(x, y, heading_deg, gap_mm=NOMINAL_BAY_MM,
              length_mm=WALL_LENGTH_MM, thickness_mm=WALL_THICKNESS_MM):
    """
    The two magenta blades as CORNER LISTS, so the browser can draw a bay that
    has been turned off the wall.

    (x, y) is the middle of the parking space itself and `heading_deg` is the
    way it opens. Placed on a wall at that wall's opening heading this
    reproduces parking.wall_rects() exactly - selftest() checks that it does,
    because two pieces of geometry for one bay is one too many and the one in
    parking.py is the one the robot uses.

    I/O:
        return: [[(x, y) x4], [(x, y) x4]] - the two walls
    """
    radians = math.radians(heading_deg)
    forward = (math.sin(radians), math.cos(radians))      # into the field
    across = (math.cos(radians), -math.sin(radians))      # along the wall
    # Centre to blade middle: half the clear gap plus half a blade, so gap_mm
    # ends up being the space between the faces.
    offset = gap_mm / 2.0 + thickness_mm / 2.0

    walls = []
    for sign in (-1.0, +1.0):
        cx = x + across[0] * sign * offset
        cy = y + across[1] * sign * offset
        walls.append([
            (cx + forward[0] * along * length_mm + across[0] * side * thickness_mm,
             cy + forward[1] * along * length_mm + across[1] * side * thickness_mm)
            for along, side in ((-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5))
        ])
    return walls


def aabb(corners):
    """A corner list as the ((x_min, y_min), (x_max, y_max)) FieldMap wants."""
    xs = [corner[0] for corner in corners]
    ys = [corner[1] for corner in corners]
    return (min(xs), min(ys)), (max(xs), max(ys))


def snap_bay(x, y, field_map):
    """
    The nearest legal bay placement to where the bay has been dragged.

    A bay is part of the mat: it sits flat against one outer wall and slides
    along it. Free placement is offered anyway - it is useful for asking "what
    if this were in the way" - but snapping is the default, because an
    unsnapped bay is a layout the field cannot be set up in, and it is the
    only form the simulator can be given (simulate() takes a section and a
    distance along it, not a pose).

    I/O:
        return: (x, y, opening heading deg, section name, along mm)
    """
    outer = field_map.outer
    sections = (("south", outer + y), ("north", outer - y),
                ("west", outer + x), ("east", outer - x))
    section = min(sections, key=lambda item: item[1])[0]
    along = y if section in ("east", "west") else x
    # Keep the whole bay inside the wall it is on.
    limit = outer - (NOMINAL_BAY_MM / 2.0 + WALL_THICKNESS_MM)
    along = max(-limit, min(limit, along))
    bay_x, bay_y, _ = bay_pose(section, along, field_map, WALL_LENGTH_MM / 2.0)
    return bay_x, bay_y, BAY_OPENING_HEADING[section], section, along


# ============================================================================
# Building the line, the follower and the planner
# ============================================================================
_CACHE = {}
_LOCK = threading.Lock()


def build(settings):
    """
    FieldMap, RacingLine, GoalPlanner and PurePursuit for one set of tunables.

    Cached on the settings themselves: a drag sends a request per frame, and
    re-sampling the racing line every time is the one avoidable cost in here.
    The key is every value, so a knob moved in the panel builds a new planner
    rather than quietly reusing the old one.
    """
    key = tuple(sorted((name, value) for name, value in settings.items()))
    found = _CACHE.get(key)
    if found is not None:
        return found

    table = defaults()

    def number(name):
        value = settings.get(name, table.get(name))
        if value is None:
            value = table.get(name)
        return None if value is None else float(value)

    field_map = FieldMap()
    path = RacingLine(field_map,
                      wall_margin_mm=(float(settings["path.wall_margin_mm"])
                                      if settings.get("path.wall_margin_mm") is not None
                                      else None),
                      corner_radius_mm=(float(settings["path.corner_radius_mm"])
                                        if settings.get("path.corner_radius_mm") is not None
                                        else None),
                      resolution_mm=number("path.resolution_mm") or 20.0)
    pursuit = PurePursuit(
        wheelbase_mm=number("pursuit.wheelbase_mm"),
        lookahead_base_mm=number("pursuit.lookahead_base_mm"),
        lookahead_per_speed_mm=number("pursuit.lookahead_per_speed_mm"),
        lookahead_min_mm=number("pursuit.lookahead_min_mm"),
        lookahead_max_mm=number("pursuit.lookahead_max_mm"),
        max_road_wheel_deg=number("pursuit.max_road_wheel_deg"),
        max_steer_command=number("pursuit.max_steer_command"),
        rear_axle_offset_mm=number("pursuit.rear_axle_offset_mm"),
        corner_lookahead_fraction=number("pursuit.corner_lookahead_fraction"))
    # min_radius_mm is FinalTask._steerable_radius_mm's arithmetic: the turning
    # circle with the caller's margin kept in hand, because a path drawn at
    # exactly full lock leaves the follower nothing to correct with.
    planner = GoalPlanner(
        field_map, path,
        min_radius_mm=pursuit.min_turn_radius_mm * number("blocks.turn_radius_margin"),
        robot_half_width_mm=number("blocks.robot_half_width_mm"),
        robot_front_mm=number("goals.robot_front_mm"),
        robot_rear_mm=number("goals.robot_rear_mm"),
        clearance_mm=number("blocks.clearance_mm"),
        min_clearance_mm=number("goals.min_clearance_mm"),
        wall_clearance_mm=number("blocks.wall_clearance_mm"),
        horizon_mm=number("goals.horizon_mm"),
        route_spacing_mm=number("goals.route_spacing_mm"),
        max_gates=int(number("goals.max_gates") or 3),
        approach_mm=number("goals.approach_mm"),
        exit_mm=number("goals.exit_mm"))

    built = (field_map, path, planner, pursuit)
    if len(_CACHE) > 60:
        _CACHE.clear()
    _CACHE[key] = built
    return built


def apply_scene(field_map, scene):
    """
    Puts the scene's bay walls on the map, and takes off any from last time.

    FieldMap obstacles are axis aligned - that is what its raycaster and
    GoalPlanner.clearances() are written against - so a bay turned off square
    goes on as the bounding box of each blade, which is bigger than the blade
    and therefore pessimistic rather than optimistic. Snapped to a wall, which
    is how a bay is really set up, the box IS the blade.

    Snapping happens here rather than in the browser, and the snapped pose is
    handed back for the browser to adopt. The alternative - snapping where the
    dragging happens - is a second copy of parking.py's geometry written in
    another language, which is the one thing this tool is trying not to have.

    I/O:
        return: (wall corner lists for drawing, the bay as it ended up)
    """
    bay = dict(scene.get("bay") or {})
    if not bay.get("enabled"):
        field_map.clear_obstacles()
        return [], bay

    x, y = float(bay.get("x", 0.0)), float(bay.get("y", 0.0))
    heading = float(bay.get("heading", 0.0))
    if bay.get("snap", True):
        x, y, heading, section, along = snap_bay(x, y, field_map)
        bay["section"], bay["along"] = section, along
    bay["x"], bay["y"], bay["heading"] = x, y, heading

    walls = bay_walls(x, y, heading,
                      gap_mm=float(bay.get("gap_mm", NOMINAL_BAY_MM)))
    field_map.set_obstacles([aabb(corners) for corners in walls])
    return walls, bay


def obstacles_of(scene):
    """The scene's pillars in the form GoalPlanner wants."""
    found = []
    for pillar in scene.get("pillars", ()):
        side = SIDE_FOR_COLOR.get(pillar.get("color"))
        if side is None:
            continue
        found.append(Obstacle(float(pillar["x"]), float(pillar["y"]), side))
    return found


# ============================================================================
# The live view
# ============================================================================

def pass_report(planner, path, plan, obstacles, direction, progress):
    """
    Per pillar: how close the swept body really gets, and WHICH SIDE it goes
    past on.

    The side is the half of it the round is scored on, and the half a single
    minimum gap cannot tell you - a plan that misses a pillar by 200mm on the
    wrong side is a fault, and reads in `pillar_gap_mm` exactly like a good
    pass. Measured off the finished geometry rather than off the goals, so a
    goal that was relaxed, turned or skipped cannot flatter the answer.

    A pillar the planner did not plan for - behind the robot, past the
    horizon, or beyond max_gates - is marked as such instead of scored. It
    would otherwise show up as a confident WRONG SIDE, which is a fault the
    round would never commit and the one thing this table must not cry wolf
    about. Which pillars those are is asked of the planner rather than worked
    out again here, so the two cannot drift apart.
    """
    report = []
    if len(plan.points) < 2:
        return report
    body = planner.footprint(plan.points, plan.headings)
    considered = {(item[3].x, item[3].y) for item in
                  planner._obstacles_ahead(progress, direction, obstacles)}

    for obstacle in obstacles:
        offsets = body - np.array([obstacle.x, obstacle.y])
        gap = float(np.min(np.hypot(offsets[:, 0], offsets[:, 1])))
        gap -= BLOCK_RADIUS_MM + planner.half_width_mm

        at, lateral = path.project(obstacle.x, obstacle.y, direction)
        index = plan.nearest_index(obstacle.x, obstacle.y)
        _, plan_lateral = path.project(float(plan.points[index][0]),
                                       float(plan.points[index][1]), direction)
        report.append({
            "x": obstacle.x, "y": obstacle.y,
            "side": obstacle.side,
            "planned": (obstacle.x, obstacle.y) in considered,
            "gap_mm": gap,
            "side_ok": bool((plan_lateral - lateral) * obstacle.side > 0.0),
            "ahead_mm": path.gap(progress, at),
            "offset_mm": plan_lateral - lateral,
        })
    return report


def goal_kinds(goals):
    """
    Labels each goal pass / exit / route.

    A pillar contributes two goals in order - the pass beside it and the exit
    past it - and they are worth telling apart on screen: the exit is the one
    that keeps the robot out until its TAIL is clear, and seeing it sit too
    close behind the pass is how you diagnose a pillar clipped on the way out.
    """
    seen, kinds = set(), []
    for goal in goals:
        if goal.obstacle is None:
            kinds.append("route")
            continue
        key = (goal.obstacle.x, goal.obstacle.y)
        kinds.append("exit" if key in seen else "pass")
        seen.add(key)
    return kinds


def plan_payload(scene):
    """
    Everything the browser draws for one layout, live.

    Always includes the VARIANT's racing line, because path.wall_margin_mm and
    path.corner_radius_mm move it and that is the most direct answer this tool
    gives to "what does this config do". The goal plan on top of it is the
    final round's business; qualification drives the line itself.
    """
    round_name = scene.get("round") or DEFAULT_ROUND
    settings = dict(scene.get("params") or {})
    field_map, path, planner, pursuit = build(settings)

    payload = {
        "ok": True,
        "round": round_name,
        "racing_line": path.points.tolist(),
        "line_length_mm": path.length,
        "corner_radius_mm": path.corner_radius,
        "line_half_mm": path.half,
        "min_radius_mm": planner.min_radius_mm,
        "turn_radius_mm": pursuit.min_turn_radius_mm,
    }

    with _LOCK:
        walls, bay = apply_scene(field_map, scene)
        car = scene.get("car") or {}
        pose = Pose(car.get("x", 0.0), car.get("y", -1050.0), car.get("heading", 90.0))
        wanted = scene.get("direction") or 0
        direction = int(wanted) if wanted in (1, -1) else path.direction_for(pose)
        progress, lateral = path.project(pose.x, pose.y, direction)
        payload.update({"direction": direction, "progress_mm": progress,
                        "lateral_mm": lateral, "bay_walls": walls, "bay": bay,
                        "lookahead_mm": pursuit.lookahead_distance(
                            max(int(settings.get("speed.base") or 70), 1))})

        if not ROUNDS[round_name]["pillars"]:
            # Qualification has no planner: the path IS the racing line, and
            # what the robot does with it is a driving question, not a
            # planning one. Press RUN.
            payload.update({"path": [], "goals": [], "body": [], "pillars": [],
                            "compromised": False, "reason": "", "plan_ms": 0.0,
                            "points": 0, "length_mm": 0.0,
                            "pillar_gap_mm": None, "wall_gap_mm": None})
            return payload

        obstacles = obstacles_of(scene)
        started = time.perf_counter()
        plan = planner.plan(pose, direction, obstacles)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        report = pass_report(planner, path, plan, obstacles, direction, progress)
        # Drawn thinned: three discs per sample at 25mm is a great many more
        # circles than a canvas needs to show where the body goes.
        body = planner.footprint(plan.points[::4], plan.headings[::4])

    payload.update({
        "path": plan.points.tolist(),
        "headings": plan.headings.tolist(),
        "goals": [{"x": goal.x, "y": goal.y, "heading": goal.heading,
                   "kind": kind, "clearance_mm": finite(goal.clearance_mm),
                   "offset_mm": goal.offset}
                  for goal, kind in zip(plan.goals, goal_kinds(plan.goals))],
        "body": body.tolist(),
        "body_radius_mm": planner.half_width_mm,
        "compromised": plan.compromised,
        "reason": plan.reason,
        "pillar_gap_mm": finite(plan.pillar_gap_mm),
        "wall_gap_mm": finite(plan.wall_gap_mm),
        "length_mm": plan.length_mm,
        "points": len(plan.points),
        "plan_ms": elapsed_ms,
        "pillars": report,
    })
    return payload


def field_payload(round_name):
    """The static geometry and this round's config, sent when a round loads."""
    settings = read_config(round_name)
    field_map, path, planner, pursuit = build(settings)
    return {
        "round": round_name,
        "rounds": {name: {"label": spec["label"], "pillars": spec["pillars"],
                          "config": str(spec["config"])}
                   for name, spec in ROUNDS.items()},
        "outer": field_map.outer,
        "inner": field_map.inner,
        "block_size_mm": BLOCK_SIZE_MM,
        "block_radius_mm": BLOCK_RADIUS_MM,
        "racing_line": path.points.tolist(),
        "line_length_mm": path.length,
        "start_zones": [{"name": name, "low": list(low), "high": list(high)}
                        for name, low, high in field_map.start_zones()],
        "settings": settings,
        "groups": [{"title": title, "keys": keys}
                   for title, keys in panel_groups(round_name)],
        "nullable": sorted(NULLABLE),
        "sim_knobs": dict(SIM_KNOBS),
        "sim_ready": simulator_status(),
        "bay": {"gap_mm": NOMINAL_BAY_MM, "length_mm": WALL_LENGTH_MM,
                "thickness_mm": WALL_THICKNESS_MM},
        "robot": {"front_mm": settings.get("goals.robot_front_mm",
                                           defaults()["goals.robot_front_mm"]),
                  "rear_mm": settings.get("goals.robot_rear_mm",
                                          defaults()["goals.robot_rear_mm"]),
                  "half_width_mm": settings.get(
                      "blocks.robot_half_width_mm",
                      defaults()["blocks.robot_half_width_mm"])},
        "planner": str(planner),
        "path_summary": str(path),
        "config": str(ROUNDS[round_name]["config"]),
    }


def finite(value):
    """inf is not JSON. Sent as None, which the panel prints as a dash."""
    if value is None:
        return None
    value = float(value)
    return None if not math.isfinite(value) else value


# ============================================================================
# Driving it: the real round, against test_driving.py's simulated robot
# ============================================================================
# One at a time, always. VirtualClock replaces time.monotonic and time.sleep
# for the WHOLE PROCESS while a run is in flight - that is what turns a 48
# second lap into two seconds of wall clock - so two runs at once would share
# one clock and both would be nonsense.
_SIM_LOCK = threading.Lock()
_SIM_MODULE = {}


def simulator():
    """
    test_driving.py, imported on demand.

    The import chain reaches the Pi's I2C library through tasks/cli.py, which
    the simulator never uses - it fakes the four things that touch metal. A
    stub stands in so the sim runs on a laptop, in the same spirit as
    utils/fake_picamera2.py. cv2 and pyserial are real packages and are asked
    for by name if they are missing, because they are a pip install away and
    guessing at why an import failed is not the user's job.
    """
    if "module" in _SIM_MODULE:
        return _SIM_MODULE["module"]
    if "error" in _SIM_MODULE:
        raise RuntimeError(_SIM_MODULE["error"])
    sys.modules.setdefault("smbus", types.ModuleType("smbus"))
    try:
        import test_driving
    except ImportError as error:
        missing = getattr(error, "name", None) or str(error)
        hint = {"cv2": "opencv-python", "serial": "pyserial"}.get(missing, missing)
        _SIM_MODULE["error"] = (
            f"driving needs `{missing}`, which is not installed - "
            f"try: pip install {hint}")
        raise RuntimeError(_SIM_MODULE["error"]) from error
    _SIM_MODULE["module"] = test_driving
    return test_driving


def simulator_status():
    """Whether RUN will work, as a message for the page."""
    try:
        simulator()
        return {"ready": True, "message": ""}
    except RuntimeError as error:
        return {"ready": False, "message": str(error)}


class OverrideConfig(TaskConfig):
    """
    A round's config.toml with the panel's edits laid over it.

    TaskConfig.set() deliberately ignores None so an unset CLI flag is a
    no-op, but here None is a value the panel can legitimately mean - unset
    path.corner_radius_mm is a different racing line from any number you could
    type. So NULLABLE keys pass None through and everything else falls back to
    the file, which keeps a box someone cleared by accident from being driven
    as `speed.base = None`.
    """

    def __init__(self, base, params):
        super().__init__(base._data, base.source)
        self._params = dict(params)

    def get(self, dotted_key, default=None):
        if dotted_key in self._params:
            value = self._params[dotted_key]
            if value is not None or dotted_key in NULLABLE:
                return value
        return super().get(dotted_key, default)


def thin(track, target=700):
    """A pose track cut down to something a canvas can draw in one frame."""
    if len(track) <= target:
        return track
    step = int(math.ceil(len(track) / float(target)))
    kept = track[::step]
    if kept[-1] is not track[-1]:
        kept.append(track[-1])
    return kept


def run_simulation(spec, run, cancelled=None):
    """
    One config, driven through its round from end to end.

    The simulator is not modified to do this. The pose track is collected by
    subclassing test_driving's own SimRobot, and a drawn pillar layout is
    given to it by standing in for place_pillars - both put back afterwards.
    Anything else would be a second copy of the run harness, and the whole
    value of this is that it is the same one test_driving.py uses.

    I/O:
        spec: the job - round, start, laps, pillars, bay, simulator knobs
        run:  {"label": ..., "params": {...}} - one config to drive
        return: metrics, the thinned pose track, and the round's own output
    """
    sim = simulator()
    round_spec = ROUNDS[spec["round"]]
    module_name, class_name = round_spec["task"]
    task_class = getattr(__import__(module_name, fromlist=[class_name]), class_name)

    params = dict(run.get("params") or {})
    params["laps.goal"] = int(spec.get("laps") or 1)
    if not spec.get("park"):
        # Parking runs after the laps and is a manoeuvre, not a line - and it
        # does not complete in the simulator today (noted in the commit that
        # added the planner). Off unless asked for, so a comparison of two
        # racing lines is not dominated by a stage neither of them reaches.
        params["parking.enabled"] = False

    base = TaskConfig.load(round_spec["config"])
    config = OverrideConfig(base, params)

    field_map = FieldMap()
    start = spec.get("start")
    if start:
        start = (float(start[0]), float(start[1]), float(start[2]))
    else:
        import random
        start = sim.random_placement(field_map, random.Random(int(spec.get("seed") or 0)))

    drawn = spec.get("drawn_pillars")
    bay = None
    if spec.get("bay"):
        bay = (spec["bay"]["section"], float(spec["bay"]["along"]))

    track = []

    class Recording(sim.SimRobot):
        """The simulator's robot, with a breadcrumb trail."""

        def advance(self, seconds, speed_command, steer_command):
            outcome = super().advance(seconds, speed_command, steer_command)
            track.append((round(self.x, 1), round(self.y, 1),
                          round(self.heading, 1), speed_command, steer_command))
            return outcome

    def stand_in_pillars(path, direction, count, rng):
        from utils.enums import Color
        colors = {"red": Color.RED, "green": Color.GREEN}
        return [sim.SimPillar(float(item["x"]), float(item["y"]),
                              colors[item["color"]],
                              path.project(float(item["x"]), float(item["y"]), 1)[0])
                for item in drawn if item.get("color") in colors]

    output = io.StringIO()
    started = time.time()
    with _SIM_LOCK:
        if cancelled is not None and cancelled.is_set():
            return None
        original_robot = sim.SimRobot
        original_place = sim.place_pillars
        sim.SimRobot = Recording
        if drawn:
            sim.place_pillars = stand_in_pillars
        try:
            with contextlib.redirect_stdout(output):
                result = sim.simulate(
                    task_class, config, start,
                    timeout_s=float(spec.get("timeout_s") or 180.0),
                    seed=int(spec.get("seed") or 0),
                    pillars=len(drawn) if drawn else int(spec.get("pillars") or 0),
                    lag_s=float(spec.get("servo_lag_s", SIM_KNOBS["servo_lag_s"])),
                    rate_deg_s=float(spec.get("servo_rate_deg_s",
                                              SIM_KNOBS["servo_rate_deg_s"])),
                    gain_error=float(spec.get("steer_gain_error",
                                              SIM_KNOBS["steer_gain_error"])),
                    bay=bay)
        finally:
            sim.SimRobot = original_robot
            sim.place_pillars = original_place
    wall_s = time.time() - started

    pillars = [{"x": pillar.x, "y": pillar.y, "color": pillar.color.value,
                "clearance_mm": finite(pillar.min_clearance_mm),
                "correct": pillar.passed_correctly}
               for pillar in result["pillars"]]
    lines = output.getvalue().splitlines()
    return {
        "label": run.get("label", ""),
        "params": run.get("params") or {},
        "completed": bool(result["completed"]),
        "crashed": bool(result["crashed"]),
        "crash_point": list(result["crash_point"]) if result["crash_point"] else None,
        "laps": result["laps"],
        "elapsed_s": result["elapsed_s"],
        "driven_mm": result["driven_mm"],
        "direction": result["direction"],
        "rms_offset_mm": result["rms_offset_mm"],
        "max_offset_mm": result["max_offset_mm"],
        "min_clearance_mm": finite(result["min_clearance_mm"]),
        "pillars": pillars,
        "pillars_wrong": len(result["pillars_wrong"]),
        "pillars_hit": len(result["pillars_hit"]),
        "start": list(start),
        "track": thin(track),
        "track_points": len(track),
        "wall_s": wall_s,
        # The round's own status output. The last of it, because the tail is
        # where a run says how it ended, and the summary lines the task prints
        # at the end are the ones worth reading first.
        "log": lines[-80:],
    }


# ============================================================================
# Jobs - a run takes seconds, so the page asks and then polls
# ============================================================================
_JOBS = {}
_JOB_IDS = itertools.count(1)
_QUEUE = queue.Queue()


def worker():
    """The one thread that drives. See _SIM_LOCK for why there is only one."""
    while True:
        job_id = _QUEUE.get()
        job = _JOBS.get(job_id)
        if job is None:
            continue
        job["state"] = "running"
        for run in job["spec"]["runs"]:
            if job["cancel"].is_set():
                job["state"] = "cancelled"
                break
            job["current"] = run.get("label", "")
            try:
                outcome = run_simulation(job["spec"], run, job["cancel"])
                if outcome is not None:
                    job["runs"].append(outcome)
            except Exception as error:                  # noqa: BLE001
                traceback.print_exc()
                job["runs"].append({"label": run.get("label", ""),
                                    "error": f"{type(error).__name__}: {error}"})
            job["done"] = len(job["runs"])
        if job["state"] != "cancelled":
            job["state"] = "finished"
        job["current"] = ""


def start_job(spec):
    """Queues a set of runs and returns the id to poll."""
    job_id = str(next(_JOB_IDS))
    _JOBS[job_id] = {"id": job_id, "spec": spec, "runs": [], "done": 0,
                     "total": len(spec["runs"]), "state": "queued",
                     "current": "", "cancel": threading.Event()}
    # Keep the last few jobs only - a track is a few thousand numbers and
    # there is no reason to hold on to every comparison ever made.
    for stale in sorted(_JOBS)[:-8]:
        if _JOBS[stale]["state"] in ("finished", "cancelled"):
            _JOBS.pop(stale, None)
    _QUEUE.put(job_id)
    return job_id


def job_payload(job_id):
    job = _JOBS.get(job_id)
    if job is None:
        return {"ok": False, "error": "no such job"}
    return {"ok": True, "id": job["id"], "state": job["state"],
            "done": job["done"], "total": job["total"],
            "current": job["current"], "runs": job["runs"]}


def sweep_runs(key, values, base_params):
    """One run per value of one knob - the panel's sweep, as a list of runs."""
    runs = []
    for raw in values:
        params = dict(base_params)
        params[key] = raw
        label = f"{key.split('.')[-1]}={'unset' if raw is None else raw}"
        runs.append({"label": label, "params": params})
    return runs


# ============================================================================
# Self-check
# ============================================================================

def selftest():
    """
    That this file's bay geometry is parking.py's, that every round's config
    loads, and that the planner still plans.

    The bay is the one place duplication was unavoidable - the tool draws a
    bay it can rotate, which parking.wall_rects() cannot express - and the
    failure mode is silent, because a bay drawn 10mm from where the planner
    was told it is looks perfectly convincing. So where the two CAN be
    compared, they are.
    """
    field_map = FieldMap()
    worst = 0.0
    for section in ("south", "east", "north", "west"):
        for along in (-900.0, -200.0, 0.0, 450.0, 1100.0):
            x, y, _ = bay_pose(section, along, field_map, WALL_LENGTH_MM / 2.0)
            mine = sorted(aabb(corners) for corners in
                          bay_walls(x, y, BAY_OPENING_HEADING[section]))
            theirs = sorted(tuple(tuple(float(v) for v in point) for point in rect)
                            for rect in wall_rects(section, along, field_map))
            for one, two in zip(mine, theirs):
                for corner_a, corner_b in zip(one, two):
                    worst = max(worst, abs(corner_a[0] - corner_b[0]),
                                abs(corner_a[1] - corner_b[1]))
    ok = worst < 1e-6
    print(f"bay walls agree with parking.wall_rects to {worst:.6f}mm  "
          f"{'OK' if ok else 'FAILED'}")

    for round_name in ROUNDS:
        payload = field_payload(round_name)
        print(f"\n[{round_name}] {payload['config']}")
        print(f"  {payload['path_summary']}")
        print(f"  {payload['planner']}")
        scene = {"round": round_name, "params": payload["settings"],
                 "car": {"x": 0.0, "y": -1050.0, "heading": 90.0},
                 "pillars": [{"x": 600.0, "y": -1050.0, "color": "red"}],
                 "bay": {"enabled": False}}
        result = plan_payload(scene)
        if ROUNDS[round_name]["pillars"]:
            print(f"  one RED pillar ahead: {result['points']} points, "
                  f"pillar gap {result['pillar_gap_mm']:.0f}mm, "
                  f"{result['plan_ms']:.1f}ms, "
                  f"{'COMPROMISED' if result['compromised'] else 'ok'}")
        else:
            print(f"  racing line {result['line_length_mm']:.0f}mm, "
                  f"corner {result['corner_radius_mm']:.0f}mm")

    status = simulator_status()
    print(f"\ndriving: {'ready' if status['ready'] else status['message']}")
    return 0 if ok else 1


# ============================================================================
# Server
# ============================================================================

class Handler(BaseHTTPRequestHandler):
    """The page, the field, a plan, and the driving jobs."""

    def log_message(self, *args):
        pass                                   # a request per drag frame

    def do_GET(self):
        route, _, query = self.path.partition("?")
        params = dict(pair.split("=", 1) for pair in query.split("&") if "=" in pair)
        if route in ("/", "/index.html"):
            if not PAGE_PATH.exists():
                return self._send(500, b"test_plan_view.html is missing", "text/plain")
            return self._send(200, PAGE_PATH.read_bytes(), "text/html; charset=utf-8")
        if route == "/field":
            round_name = params.get("round", DEFAULT_ROUND)
            if round_name not in ROUNDS:
                round_name = DEFAULT_ROUND
            return self._json(field_payload(round_name))
        if route == "/job":
            return self._json(job_payload(params.get("id", "")))
        if route == "/favicon.ico":
            # Answered rather than 404'd so the browser console stays empty: a
            # console with one harmless error in it is a console nobody reads,
            # and this page reports real ones there.
            self.send_response(204)
            self.end_headers()
            return None
        return self._send(404, b"not found", "text/plain")

    def do_POST(self):
        length = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
        except ValueError as error:
            return self._json({"ok": False, "error": f"bad request: {error}"})
        try:
            if self.path == "/plan":
                return self._json(plan_payload(body))
            if self.path == "/sim":
                status = simulator_status()
                if not status["ready"]:
                    return self._json({"ok": False, "error": status["message"]})
                return self._json({"ok": True, "id": start_job(body)})
            if self.path == "/cancel":
                job = _JOBS.get(body.get("id", ""))
                if job is not None:
                    job["cancel"].set()
                return self._json({"ok": True})
        except Exception as error:                      # noqa: BLE001
            # A layout or a config that breaks something is a RESULT, not a
            # crash: the browser shows the error and stays usable, which is
            # what you want when you have just typed the number that broke it
            # and would rather not lose everything else you set up.
            traceback.print_exc()
            return self._json({"ok": False,
                               "error": f"{type(error).__name__}: {error}"})
        return self._send(404, b"not found", "text/plain")

    def _json(self, payload):
        self._send(200, json.dumps(payload).encode("utf-8"), "application/json")

    def _send(self, code, body, content_type):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Interactive field view for the qualification and final rounds.")
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument("--host", default="127.0.0.1",
                        help="127.0.0.1 keeps it on this machine; 0.0.0.0 "
                             "serves it to the network, e.g. from the Pi")
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--selftest", action="store_true",
                        help="check the geometry and both configs, then exit")
    args = parser.parse_args(argv)

    if args.selftest:
        return selftest()

    for round_name in ROUNDS:
        payload = field_payload(round_name)
        print(f"[{round_name}] {payload['path_summary']}")
    status = simulator_status()
    print(f"driving: {'ready' if status['ready'] else status['message']}")

    threading.Thread(target=worker, daemon=True).start()
    url = f"http://{'127.0.0.1' if args.host == '0.0.0.0' else args.host}:{args.port}/"
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"\n  {url}\n\nCtrl-C to stop.")
    if not args.no_browser:
        threading.Timer(0.4, webbrowser.open, [url]).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

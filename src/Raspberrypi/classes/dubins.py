"""
Dubins paths: the shortest route a car that cannot reverse can drive from one
POSE to another.

This is the primitive the goal-based planner is built on. Give it "I am here,
facing this way" and "I want to be there, facing that way" and it returns the
shortest path between them that never asks the steering for more lock than the
robot has. That is the whole difference between a goal the robot can actually
reach and a goal it merely points at.

A Dubins path is always three segments drawn from {Left arc, Straight, Right
arc} - LSL, RSR, LSR, RSL, RLR or LRL. Six closed-form candidates, no search,
no iteration: solving one costs microseconds, which is what makes it usable
inside a control loop on a Pi rather than something run once offline.

    +--------------------------------------------------+
    |   start ->---.                                   |
    |               \\  R                    goal       |
    |                `----------------->----.          |
    |                        S               \\  L     |
    |                                         `->      |
    +--------------------------------------------------+

WHAT IT CANNOT DO, so the caller does not have to find out the hard way:

  * It does not know about obstacles. It returns the shortest path, which may
    drive straight through a pillar. Collision checking is the planner's job
    (see GoalPlanner.check).
  * It does not reverse. That is deliberate here - the round is driven
    forwards and a plan that needs a three-point turn is a plan that has
    already gone wrong. Reeds-Shepp is the variant that allows reverse, and
    the parking manoeuvre has its own controller for that (classes/parking.py).

--------------------------------------------------------------------------
The six-candidate solver below (_LSL through _LRL, _dubins_path_planning_from
_origin, _interpolate, _generate_local_course) is taken from PythonRobotics
by Atsushi Sakai, MIT licensed:

    https://github.com/AtsushiSakai/PythonRobotics
    PathPlanning/DubinsPath/dubins_path_planner.py
    Copyright (c) 2016 - now Atsushi Sakai and contributors

Two changes were made to it, both mechanical:

  * `rot_mat_2d` and `angle_mod` came from PythonRobotics' own utils and
    pulled in scipy for what is a 2x2 rotation. They are inlined below in
    plain numpy, so this file adds no dependency the robot did not already
    have - scipy is a long build on a Pi and none of the rest of it is used.
  * The matplotlib demo `main()` was dropped.

The maths is untouched. `plan_dubins` at the bottom is ours: it is the
adapter between that solver's convention (metres-agnostic, yaw in radians
counter-clockwise from +X) and this robot's (millimetres, heading in degrees
CLOCKWISE from +Y - see field_map.py).
--------------------------------------------------------------------------
"""
import math
from collections import namedtuple
from math import acos, atan2, cos, hypot, pi, sin, sqrt

import numpy as np

# A planned path, sampled. `points` is Nx2 in field mm, `headings` is N in
# degrees clockwise from +Y, and `length` is the true arc length in mm - not
# the polyline length, which is shorter by however coarse `step_mm` was.
DubinsPath = namedtuple("DubinsPath", "points headings length modes")


# ============================================================================
# PythonRobotics solver (MIT - see module docstring)
# ============================================================================

def _angle_mod(x, zero_2_2pi=False):
    """Wraps to [-pi, pi), or to [0, 2pi) with `zero_2_2pi`."""
    if zero_2_2pi:
        return x % (2.0 * pi)
    return (x + pi) % (2.0 * pi) - pi


def _rot_mat_2d(angle):
    """2D rotation matrix. Replaces PythonRobotics' scipy-backed version."""
    cosine, sine = cos(angle), sin(angle)
    return np.array([[cosine, -sine], [sine, cosine]])


def _mod2pi(theta):
    return _angle_mod(theta, zero_2_2pi=True)


def _calc_trig_funcs(alpha, beta):
    sin_a = sin(alpha)
    sin_b = sin(beta)
    cos_a = cos(alpha)
    cos_b = cos(beta)
    cos_ab = cos(alpha - beta)
    return sin_a, sin_b, cos_a, cos_b, cos_ab


def _LSL(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    mode = ["L", "S", "L"]
    p_squared = 2 + d ** 2 - (2 * cos_ab) + (2 * d * (sin_a - sin_b))
    if p_squared < 0:  # invalid configuration
        return None, None, None, mode
    tmp = atan2((cos_b - cos_a), d + sin_a - sin_b)
    d1 = _mod2pi(-alpha + tmp)
    d2 = sqrt(p_squared)
    d3 = _mod2pi(beta - tmp)
    return d1, d2, d3, mode


def _RSR(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    mode = ["R", "S", "R"]
    p_squared = 2 + d ** 2 - (2 * cos_ab) + (2 * d * (sin_b - sin_a))
    if p_squared < 0:
        return None, None, None, mode
    tmp = atan2((cos_a - cos_b), d - sin_a + sin_b)
    d1 = _mod2pi(alpha - tmp)
    d2 = sqrt(p_squared)
    d3 = _mod2pi(-beta + tmp)
    return d1, d2, d3, mode


def _LSR(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    p_squared = -2 + d ** 2 + (2 * cos_ab) + (2 * d * (sin_a + sin_b))
    mode = ["L", "S", "R"]
    if p_squared < 0:
        return None, None, None, mode
    d1 = sqrt(p_squared)
    tmp = atan2((-cos_a - cos_b), (d + sin_a + sin_b)) - atan2(-2.0, d1)
    d2 = _mod2pi(-alpha + tmp)
    d3 = _mod2pi(-_mod2pi(beta) + tmp)
    return d2, d1, d3, mode


def _RSL(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    p_squared = d ** 2 - 2 + (2 * cos_ab) - (2 * d * (sin_a + sin_b))
    mode = ["R", "S", "L"]
    if p_squared < 0:
        return None, None, None, mode
    d1 = sqrt(p_squared)
    tmp = atan2((cos_a + cos_b), (d - sin_a - sin_b)) - atan2(2.0, d1)
    d2 = _mod2pi(alpha - tmp)
    d3 = _mod2pi(beta - tmp)
    return d2, d1, d3, mode


def _RLR(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    mode = ["R", "L", "R"]
    tmp = (6.0 - d ** 2 + 2.0 * cos_ab + 2.0 * d * (sin_a - sin_b)) / 8.0
    if abs(tmp) > 1.0:
        return None, None, None, mode
    d2 = _mod2pi(2 * pi - acos(tmp))
    d1 = _mod2pi(alpha - atan2(cos_a - cos_b, d - sin_a + sin_b) + d2 / 2.0)
    d3 = _mod2pi(alpha - beta - d1 + d2)
    return d1, d2, d3, mode


def _LRL(alpha, beta, d):
    sin_a, sin_b, cos_a, cos_b, cos_ab = _calc_trig_funcs(alpha, beta)
    mode = ["L", "R", "L"]
    tmp = (6.0 - d ** 2 + 2.0 * cos_ab + 2.0 * d * (- sin_a + sin_b)) / 8.0
    if abs(tmp) > 1.0:
        return None, None, None, mode
    d2 = _mod2pi(2 * pi - acos(tmp))
    d1 = _mod2pi(-alpha - atan2(cos_a - cos_b, d + sin_a - sin_b) + d2 / 2.0)
    d3 = _mod2pi(_mod2pi(beta) - alpha - d1 + _mod2pi(d2))
    return d1, d2, d3, mode


_PATH_TYPE_MAP = {"LSL": _LSL, "RSR": _RSR, "LSR": _LSR, "RSL": _RSL,
                  "RLR": _RLR, "LRL": _LRL}


def _interpolate(length, mode, max_curvature, origin_x, origin_y,
                 origin_yaw, path_x, path_y, path_yaw):
    if mode == "S":
        path_x.append(origin_x + length / max_curvature * cos(origin_yaw))
        path_y.append(origin_y + length / max_curvature * sin(origin_yaw))
        path_yaw.append(origin_yaw)
    else:  # curve
        ldx = sin(length) / max_curvature
        ldy = 0.0
        if mode == "L":  # left turn
            ldy = (1.0 - cos(length)) / max_curvature
        elif mode == "R":  # right turn
            ldy = (1.0 - cos(length)) / -max_curvature
        gdx = cos(-origin_yaw) * ldx + sin(-origin_yaw) * ldy
        gdy = -sin(-origin_yaw) * ldx + cos(-origin_yaw) * ldy
        path_x.append(origin_x + gdx)
        path_y.append(origin_y + gdy)

        if mode == "L":  # left turn
            path_yaw.append(origin_yaw + length)
        elif mode == "R":  # right turn
            path_yaw.append(origin_yaw - length)

    return path_x, path_y, path_yaw


def _generate_local_course(lengths, modes, max_curvature, step_size):
    p_x, p_y, p_yaw = [0.0], [0.0], [0.0]

    for (mode, length) in zip(modes, lengths):
        if length == 0.0:
            continue

        # set origin state
        origin_x, origin_y, origin_yaw = p_x[-1], p_y[-1], p_yaw[-1]

        current_length = step_size
        while abs(current_length + step_size) <= abs(length):
            p_x, p_y, p_yaw = _interpolate(current_length, mode, max_curvature,
                                           origin_x, origin_y, origin_yaw,
                                           p_x, p_y, p_yaw)
            current_length += step_size

        p_x, p_y, p_yaw = _interpolate(length, mode, max_curvature, origin_x,
                                       origin_y, origin_yaw, p_x, p_y, p_yaw)

    return p_x, p_y, p_yaw


def _dubins_path_planning_from_origin(end_x, end_y, end_yaw, curvature,
                                      step_size, planning_funcs):
    dx = end_x
    dy = end_y
    d = hypot(dx, dy) * curvature

    theta = _mod2pi(atan2(dy, dx))
    alpha = _mod2pi(-theta)
    beta = _mod2pi(end_yaw - theta)

    best_cost = float("inf")
    b_d1, b_d2, b_d3, b_mode = None, None, None, None

    for planner in planning_funcs:
        d1, d2, d3, mode = planner(alpha, beta, d)
        if d1 is None:
            continue

        cost = (abs(d1) + abs(d2) + abs(d3))
        if best_cost > cost:  # Select minimum length one.
            b_d1, b_d2, b_d3, b_mode, best_cost = d1, d2, d3, mode, cost

    if b_mode is None:
        return None, None, None, None, None

    lengths = [b_d1, b_d2, b_d3]
    x_list, y_list, yaw_list = _generate_local_course(lengths, b_mode,
                                                      curvature, step_size)

    lengths = [length / curvature for length in lengths]

    return x_list, y_list, yaw_list, b_mode, lengths


def plan_dubins_path(s_x, s_y, s_yaw, g_x, g_y, g_yaw, curvature,
                     step_size=0.1, selected_types=None):
    """
    Shortest curvature-bounded path from one pose to another.

    Distances are in whatever unit the caller uses consistently; `curvature`
    is 1/radius in that same unit. Yaw is radians, counter-clockwise from +X.

    I/O:
        return: (x_list, y_list, yaw_list, modes, lengths), or five Nones if
                no candidate solved - see plan_dubins for the wrapper that
                turns that into a clean None.
    """
    if selected_types is None:
        planning_funcs = _PATH_TYPE_MAP.values()
    else:
        planning_funcs = [_PATH_TYPE_MAP[ptype] for ptype in selected_types]

    # calculate local goal x, y, yaw
    l_rot = _rot_mat_2d(s_yaw)
    le_xy = np.stack([g_x - s_x, g_y - s_y]).T @ l_rot
    local_goal_x = le_xy[0]
    local_goal_y = le_xy[1]
    local_goal_yaw = g_yaw - s_yaw

    lp_x, lp_y, lp_yaw, modes, lengths = _dubins_path_planning_from_origin(
        local_goal_x, local_goal_y, local_goal_yaw, curvature, step_size,
        planning_funcs)
    if modes is None:
        return None, None, None, None, None

    # Convert a local coordinate path to the global coordinate
    rot = _rot_mat_2d(-s_yaw)
    converted_xy = np.stack([lp_x, lp_y]).T @ rot
    x_list = converted_xy[:, 0] + s_x
    y_list = converted_xy[:, 1] + s_y
    yaw_list = _angle_mod(np.array(lp_yaw) + s_yaw)

    return x_list, y_list, yaw_list, modes, lengths


# ============================================================================
# This robot's convention
# ============================================================================
# The solver above works in the usual maths convention: yaw counter-clockwise
# from +X, so the unit vector for yaw t is (cos t, sin t). Everything else in
# this codebase measures heading CLOCKWISE from +Y, so its unit vector is
# (sin h, cos h) - see field_map.py. Equating the two gives h = 90 - t, and
# because that mapping is its own inverse the same expression converts both
# ways. It is written out twice anyway: a sign error here is a robot that
# mirrors every plan, which is a miserable thing to debug on a mat.

def heading_to_yaw(heading_deg):
    """Degrees clockwise from +Y -> radians counter-clockwise from +X."""
    return math.radians(90.0 - heading_deg)


def yaw_to_heading(yaw_rad):
    """Radians counter-clockwise from +X -> degrees clockwise from +Y."""
    return (90.0 - math.degrees(yaw_rad)) % 360.0


def plan_dubins(start, goal, min_radius_mm, step_mm=25.0):
    """
    Shortest forward-only path from `start` to `goal`, in this robot's frame.

    I/O:
        start, goal: (x_mm, y_mm, heading_deg), heading clockwise from +Y
        min_radius_mm: the tightest arc the plan may contain. Pass the
                       robot's own turning circle with a margin on it -
                       a path drawn at exactly full lock leaves the follower
                       nothing to correct tracking error with.
        step_mm: sampling interval along the returned polyline
        return: DubinsPath, or None if no path exists (which for Dubins means
                the geometry was degenerate - start and goal on top of each
                other - rather than "too hard")

    The returned `length` is the true arc length, so it can be compared
    against a lookahead or used to choose between candidate goals without
    caring how finely it was sampled.
    """
    if min_radius_mm <= 0.0:
        raise ValueError("min_radius_mm must be positive")

    curvature = 1.0 / float(min_radius_mm)
    x_list, y_list, yaw_list, modes, lengths = plan_dubins_path(
        float(start[0]), float(start[1]), heading_to_yaw(start[2]),
        float(goal[0]), float(goal[1]), heading_to_yaw(goal[2]),
        curvature, step_size=float(step_mm) * curvature)
    if modes is None:
        return None

    points = np.column_stack((x_list, y_list))
    headings = np.array([yaw_to_heading(yaw) for yaw in yaw_list])
    return DubinsPath(points=points, headings=headings,
                      length=float(sum(abs(value) for value in lengths)),
                      modes="".join(modes))


def dubins_cost(start, goal, min_radius_mm):
    """
    How long the path is and how far it turns, WITHOUT sampling it.

    Both numbers come straight out of the solver's three segment lengths, so
    this costs about a third of what plan_dubins does. That matters: the
    planner screens dozens of candidate goals per pillar and only ever draws
    the one that wins.

    The turn total is the loop detector. A Dubins path's arc segments turn
    through length/radius radians each, so summing those is exact - no
    sampling, no unwrapping, no threshold on how finely the path was drawn.
    A path that turns through more than about a half-circle more than the
    heading change asked for is going the long way round, which is what the
    solver returns when a goal cannot be reached in the room available.

    I/O:
        return: (length_mm, turn_deg), or (inf, inf) if no path exists
    """
    curvature = 1.0 / float(min_radius_mm)
    _, _, _, modes, lengths = plan_dubins_path(
        float(start[0]), float(start[1]), heading_to_yaw(start[2]),
        float(goal[0]), float(goal[1]), heading_to_yaw(goal[2]),
        curvature, step_size=1e9)
    if modes is None:
        return float("inf"), float("inf")

    length = sum(abs(value) for value in lengths)
    turn = sum(abs(value) * curvature
               for mode, value in zip(modes, lengths) if mode != "S")
    return float(length), math.degrees(turn)


def dubins_length(start, goal, min_radius_mm):
    """Just the length - see dubins_cost."""
    return dubins_cost(start, goal, min_radius_mm)[0]

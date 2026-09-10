#!/usr/bin/env python3
# waypoints.py — path/pattern generation + CSV I/O for autonomous missions.
#
# Every pattern reduces to a list of (x, y, speed_mps) in a LOCAL,
# compass-referenced Cartesian frame (meters) — see odometry.py's module
# docstring for what "compass-referenced" means here. x-axis = the
# mission's reference heading (0 deg), y-axis = 90 deg left of it
# (CCW-positive, matching the Ackermann sign convention already documented
# in ackermann_config.h: positive angle = left turn).
#
# No RTK/GPS involved anywhere in this file — these patterns are pure
# geometry, unlike Old_files/UGV_closed/generate_straight_line.py and
# generate_rectangle.py, which needed real lat/lon survey points. This
# rover has no RTK wired to the RPi at all; position comes from dead
# reckoning only (odometry.py) — see architecture.md / scratchpad.md.

import csv
import math

import rover_config as cfg


def save_csv(waypoints, path):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "speed_mps"])
        for x, y, speed in waypoints:
            writer.writerow([f"{x:.3f}", f"{y:.3f}", f"{speed:.3f}"])


def load_csv(path):
    waypoints = []
    with open(path, "r") as f:
        reader = csv.reader(f)
        next(reader)  # header
        for row in reader:
            waypoints.append((float(row[0]), float(row[1]), float(row[2])))
    return waypoints


def _wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def generate_straight_line(length_m, spacing=cfg.SPACING_STRAIGHT_M,
                            speed=cfg.CRUISE_SPEED_MPS):
    """Straight line from (0,0) along +X. Endpoints-only would be enough
    for Pure Pursuit (guidance.py finds points ALONG a segment
    analytically), but interpolated at `spacing` anyway to match the old
    CSV convention (Old_files/UGV_closed/waypoints_1m.csv) and give
    consistent progress/logging granularity."""
    n = max(1, round(length_m / spacing))
    return [(i * length_m / n, 0.0, speed) for i in range(n + 1)]


def generate_rectangle(width_m, height_m, spacing=cfg.SPACING_STRAIGHT_M,
                        speed=cfg.CRUISE_SPEED_MPS, closed=True):
    """Rectangle perimeter starting at (0,0), first edge along +X:
    (0,0) -> (width,0) -> (width,height) -> (0,height) [-> back to (0,0)]."""
    corners = [(0.0, 0.0), (width_m, 0.0), (width_m, height_m), (0.0, height_m)]
    if closed:
        corners.append(corners[0])

    waypoints = []
    for i in range(len(corners) - 1):
        ax, ay = corners[i]
        bx, by = corners[i + 1]
        seg_len = math.hypot(bx - ax, by - ay)
        n = max(1, round(seg_len / spacing))
        for j in range(n):
            t = j / n
            waypoints.append((ax + t * (bx - ax), ay + t * (by - ay), speed))
    waypoints.append((corners[-1][0], corners[-1][1], speed))
    return waypoints


def generate_circle(diameter_m, spacing=cfg.SPACING_CURVE_M,
                     speed=cfg.CRUISE_SPEED_MPS, ccw=True):
    """Full circle, starting at (0,0) heading +X, tangent to the circle at
    the start (center placed directly to the side so no initial jump is
    needed to join the path)."""
    radius = diameter_m / 2.0
    if radius < cfg.MIN_TURN_RADIUS_M:
        print(f"[waypoints] WARNING: circle radius {radius:.2f}m is tighter "
              f"than the chassis's minimum turn radius "
              f"({cfg.MIN_TURN_RADIUS_M:.2f}m at full steering lock) — this "
              f"path is not actually drivable as generated.")

    sign = 1.0 if ccw else -1.0
    center_y = sign * radius
    circumference = 2 * math.pi * radius
    n = max(3, round(circumference / spacing))

    waypoints = []
    for i in range(n + 1):  # +1 point to close the loop back at the start
        theta = sign * (2 * math.pi * i / n) - sign * (math.pi / 2)
        x = radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)
        waypoints.append((x, y, speed))
    return waypoints


def _semicircle_turn(end_x, end_y, heading_rad, row_spacing, side,
                      curve_spacing=cfg.SPACING_CURVE_M):
    """Connects the end of one lawnmower row to the start of the next with
    a semicircular U-turn, curving toward `side` (+1 = left, -1 = right).

    Uses the CHASSIS'S ACTUAL minimum turn radius, not row_spacing/2 — a
    clean single-arc U-turn needs radius = row_spacing/2 to land exactly
    row_spacing over. If that radius is tighter than the vehicle can
    physically steer, it's not achievable with a single arc at all; see
    the WARNING this prints and scratchpad.md for the real situation this
    project hit (row_spacing=1.0m wants a 0.5m-radius turn, chassis
    minimum is 0.8m)."""
    ideal_radius = row_spacing / 2.0
    radius = max(ideal_radius, cfg.MIN_TURN_RADIUS_M)
    if radius > ideal_radius + 1e-6:
        print(f"[waypoints] WARNING: row_spacing={row_spacing:.2f}m wants a "
              f"{ideal_radius:.2f}m-radius U-turn, tighter than this "
              f"chassis's {cfg.MIN_TURN_RADIUS_M:.2f}m minimum turn radius "
              f"— using {radius:.2f}m instead. This turn lands "
              f"{2*radius:.2f}m over, not {row_spacing:.2f}m, and needs "
              f"~{radius:.2f}m of headland clearance beyond the row end. "
              f"A tight in-spacing turn would need a reverse "
              f"(three-point-turn style) maneuver — not implemented yet.")

    # Turn center: offset from the row-end point, perpendicular to travel,
    # toward `side` (left = CCW-positive, matching the project's sign
    # convention).
    cx = end_x - side * radius * math.sin(heading_rad)
    cy = end_y + side * radius * math.cos(heading_rad)

    start_angle = math.atan2(end_y - cy, end_x - cx)
    n = max(3, round((math.pi * radius) / curve_spacing))

    points = []
    for i in range(1, n + 1):
        frac = i / n
        angle = start_angle + side * math.pi * frac
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        points.append((x, y))

    final_x, final_y = points[-1]
    return points, final_x, final_y


def generate_lawnmower(row_length_m, num_rows,
                        row_spacing=cfg.LAWNMOWER_ROW_SPACING_M,
                        speed=cfg.CRUISE_SPEED_MPS,
                        curve_spacing=cfg.SPACING_CURVE_M,
                        straight_spacing=cfg.SPACING_STRAIGHT_M):
    """Boustrophedon coverage pattern: `num_rows` parallel passes of
    `row_length_m` each, `row_spacing` apart, connected by U-turns
    (_semicircle_turn — at row_spacing=1.0m on this chassis, turns land
    wider than row_spacing; see the printed warning). Turn direction
    alternates each row so the pattern doesn't drift sideways."""
    waypoints = [(0.0, 0.0, speed)]
    x, y = 0.0, 0.0
    heading = 0.0   # radians; alternates 0 / pi each row
    side = 1        # alternates turn direction

    for row in range(num_rows):
        dx = math.cos(heading) * row_length_m
        dy = math.sin(heading) * row_length_m
        n = max(1, round(row_length_m / straight_spacing))
        for j in range(1, n + 1):
            t = j / n
            waypoints.append((x + t * dx, y + t * dy, speed))
        x, y = x + dx, y + dy

        if row < num_rows - 1:
            turn_pts, x, y = _semicircle_turn(x, y, heading, row_spacing,
                                               side, curve_spacing)
            for px, py in turn_pts:
                waypoints.append((px, py, speed))
            heading = _wrap_to_pi(heading + math.pi)
            side *= -1

    return waypoints


if __name__ == "__main__":
    # Quick standalone sanity check — no hardware needed, just prints
    # waypoint counts for each pattern so the math can be eyeballed before
    # ever touching the actual rover.
    for name, wps in [
        ("straight_line(10m)", generate_straight_line(10.0)),
        ("rectangle(8x6)", generate_rectangle(8.0, 6.0)),
        ("circle(d=6m)", generate_circle(6.0)),
        ("lawnmower(6m rows x4)", generate_lawnmower(6.0, 4)),
    ]:
        print(f"{name}: {len(wps)} waypoints, "
              f"start={wps[0][:2]}, end={wps[-1][:2]}")

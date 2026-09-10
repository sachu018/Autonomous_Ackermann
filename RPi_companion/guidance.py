#!/usr/bin/env python3
# guidance.py — Pure Pursuit path-following controller (Phase 5).
#
# Takes the current dead-reckoned Pose (odometry.py) and a path (list of
# (x, y, speed) waypoints from waypoints.py), produces the next
# (steer_target_deg, speed_target_ms) command to hand to
# uart_link.STM32Link.send_command() — this is its ONLY output; this
# module never touches the STM32 link or any hardware directly.
#
# Unlike Old_files/UGV_closed/p_controller.py's segment-local lookahead
# (project onto the CURRENT segment only, then step Ld along it), this
# searches forward across the whole path to find where a circle of radius
# Ld centered on the rover actually intersects it — the textbook Pure
# Pursuit geometry. That's necessary here specifically because curve
# waypoint spacing (0.3 m) is narrower than Ld (0.8 m), so on the circle/
# lawnmower-turn sections the lookahead point routinely spans several
# waypoints at once — the old segment-local approach would need the
# lookahead point to still be within the CURRENT segment, which it won't
# be. See scratchpad.md for this reasoning.

import math

import rover_config as cfg


def _wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PurePursuit:
    def __init__(self, waypoints,
                 lookahead_m=cfg.LOOKAHEAD_M,
                 wp_thresh_m=cfg.WP_REACHED_THRESH_M,
                 wheelbase_m=cfg.WHEELBASE_M,
                 max_steer_deg=cfg.MAX_STEER_ANGLE_DEG,
                 yaw_bound_deg=cfg.YAW_BOUND_DEG,
                 speed_cos_floor=cfg.SPEED_COS_FLOOR):
        if len(waypoints) < 2:
            raise ValueError("PurePursuit needs at least 2 waypoints")

        self.waypoints = waypoints  # list of (x, y, speed)
        self.Ld = lookahead_m
        self.wp_thresh = wp_thresh_m
        self.L = wheelbase_m
        self.max_steer_deg = max_steer_deg
        self.yaw_bound = math.radians(yaw_bound_deg)
        self.speed_cos_floor = speed_cos_floor

        self._nearest_idx = 0  # forward-only search cursor
        self._finished = False

    def is_finished(self):
        return self._finished

    def _advance_nearest_index(self, x, y):
        """Moves the search cursor forward past waypoints the rover has
        already effectively reached, so the lookahead search doesn't latch
        onto a point behind the rover on a path that passes near itself
        again later (e.g. a lawnmower turn looping back close to an
        earlier row)."""
        n = len(self.waypoints)
        while self._nearest_idx < n - 1:
            wx, wy, _ = self.waypoints[self._nearest_idx]
            if math.hypot(wx - x, wy - y) > self.wp_thresh:
                break
            self._nearest_idx += 1

    def _find_lookahead_point(self, x, y):
        """Walks forward from the cursor, returns the point where a circle
        of radius Ld centered on the rover intersects the path — the
        furthest-along intersection found, so the rover always aims
        forward. Falls back to the final waypoint if the whole remaining
        path is already closer than Ld (finishing approach)."""
        n = len(self.waypoints)
        for i in range(self._nearest_idx, n - 1):
            ax, ay, _ = self.waypoints[i]
            bx, by, b_speed = self.waypoints[i + 1]

            dx, dy = bx - ax, by - ay
            fx, fy = ax - x, ay - y

            a_coef = dx * dx + dy * dy
            if a_coef < 1e-9:
                continue  # degenerate (duplicate) waypoint, skip

            b_coef = 2.0 * (fx * dx + fy * dy)
            c_coef = fx * fx + fy * fy - self.Ld * self.Ld

            disc = b_coef * b_coef - 4.0 * a_coef * c_coef
            if disc < 0.0:
                continue  # this segment never reaches Ld from the rover

            disc_sqrt = math.sqrt(disc)
            t_far = (-b_coef + disc_sqrt) / (2.0 * a_coef)
            t_near = (-b_coef - disc_sqrt) / (2.0 * a_coef)

            for t in (t_far, t_near):
                if 0.0 <= t <= 1.0:
                    return ax + t * dx, ay + t * dy, b_speed

        fx, fy, f_speed = self.waypoints[-1]
        return fx, fy, f_speed

    def update(self, pose):
        """pose: odometry.Pose. Returns (steer_target_deg, speed_target_ms).
        Returns (0.0, 0.0) once is_finished() is True — caller should stop
        calling update() and hand off to whatever "mission complete"
        handling it wants at that point."""
        x, y, yaw = pose.x, pose.y, pose.yaw

        fx, fy, _ = self.waypoints[-1]
        if math.hypot(fx - x, fy - y) < self.wp_thresh:
            self._finished = True
            return 0.0, 0.0

        self._advance_nearest_index(x, y)
        lx, ly, target_speed = self._find_lookahead_point(x, y)

        dx, dy = lx - x, ly - y
        target_bearing = math.atan2(dy, dx)
        alpha = _wrap_to_pi(target_bearing - yaw)

        ld_actual = max(math.hypot(dx, dy), 0.05)  # guard divide-by-zero
        curvature = (2.0 * math.sin(alpha)) / ld_actual
        steer_deg = math.degrees(math.atan(curvature * self.L))
        steer_deg = max(-self.max_steer_deg, min(self.max_steer_deg, steer_deg))

        # Speed shaping — same formula as the old P controller
        # (Old_files/UGV_closed/p_controller.py): bound the heading error
        # before using it to shape speed, then slow down toward sharp
        # turns. Kept as the old defaults for now; retune from field-
        # measured cross-track error later (see scratchpad.md).
        alpha_bounded = max(-self.yaw_bound, min(self.yaw_bound, alpha))
        speed = target_speed * max(self.speed_cos_floor, math.cos(alpha_bounded))
        speed = max(cfg.MIN_SPEED_MPS, speed)

        return steer_deg, speed


if __name__ == "__main__":
    # Standalone sanity check with a synthetic straight-line path and a
    # rover starting slightly off-axis — no hardware needed. Confirms the
    # controller steers back toward the line and terminates at the end.
    from odometry import Pose
    import waypoints as wp

    path = wp.generate_straight_line(10.0)
    pp = PurePursuit(path)

    pose = Pose(x=0.0, y=0.5, yaw=0.0)  # 0.5m off to the side, facing +X
    ticks = 0
    while not pp.is_finished() and ticks < 2000:
        steer_deg, speed = pp.update(pose)
        # Crude kinematic step for this smoke test only (NOT the real
        # bicycle model) — just enough to confirm convergence direction.
        pose.yaw += math.radians(steer_deg) * 0.05 * 0.5
        pose.x += speed * math.cos(pose.yaw) * 0.05
        pose.y += speed * math.sin(pose.yaw) * 0.05
        ticks += 1

    print(f"Finished after {ticks} ticks: pose=({pose.x:.2f}, {pose.y:.2f}), "
          f"expected y to have converged toward 0 (started at 0.5)")

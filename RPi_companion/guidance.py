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

        # Diagnostics from the most recent update() call — for logging/
        # tuning only, never fed back into the steering law itself. None
        # until the first update().
        self.last_lookahead = None   # (x, y)
        self.last_alpha_deg = None   # heading error to the lookahead point
        self.last_cte_m = None       # signed cross-track error, +left of path
        self.last_target_idx = 0     # nearest_idx at the time of that update

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
        path is already closer than Ld (finishing approach).

        Starts the search ONE segment behind the cursor, not AT it —
        found on real hardware: a mission always starts with the rover
        sitting exactly on waypoint 0 (pose is defined to be (0,0) =
        waypoints[0] at mission start), so _advance_nearest_index()
        immediately advances the cursor to 1 on the very first tick
        (distance 0 < wp_thresh). Starting the search exactly AT the
        cursor then skips segment 0->1 entirely — precisely the segment
        that should have matched — and the search fell through every
        later segment (genuinely outside Ld from the origin) straight to
        the "aim at the final waypoint" fallback. The smoke test in
        __main__ didn't catch this because it started 0.5m off-axis,
        never landing exactly on a waypoint. See scratchpad.md."""
        n = len(self.waypoints)
        start_i = max(0, self._nearest_idx - 1)
        for i in range(start_i, n - 1):
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

    def _cross_track_error(self, x, y):
        """Signed perpendicular distance from (x,y) to the path segment at
        the tracking cursor — the actual tuning metric (matches the old
        BBB/UGV_closed systems' "CTE" their tuned gains were measured
        against, e.g. ismc_controller.py's `ey`). NOT used in the steering
        law itself, diagnostic only. Positive = path is to the rover's
        left (consistent with this project's CCW-positive convention).
        Uses the same "one segment behind the cursor" adjustment as
        _find_lookahead_point, for the same reason — see its docstring."""
        idx = max(0, min(self._nearest_idx - 1, len(self.waypoints) - 2))
        ax, ay, _ = self.waypoints[idx]
        bx, by, _ = self.waypoints[idx + 1]
        dx, dy = bx - ax, by - ay
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-9:
            return 0.0
        cross = dx * (y - ay) - dy * (x - ax)
        return cross / math.sqrt(seg_len_sq)

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

        # Record diagnostics for the caller to log — see class docstring
        # attributes above. Computed AFTER _advance_nearest_index() so the
        # CTE reflects the current cursor.
        self.last_lookahead = (lx, ly)
        self.last_alpha_deg = math.degrees(alpha)
        self.last_cte_m = self._cross_track_error(x, y)
        self.last_target_idx = self._nearest_idx

        return steer_deg, speed


if __name__ == "__main__":
    from odometry import Pose
    import waypoints as wp

    # Check 1: rover starting slightly off-axis — confirms the controller
    # steers back toward the line and terminates at the end.
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
    print(f"Check 1 (0.5m off-axis start): finished after {ticks} ticks, "
          f"pose=({pose.x:.2f}, {pose.y:.2f}) (expect y converged toward 0)")

    # Check 2: rover starting EXACTLY on waypoint 0 — the actual real-
    # mission case (mission.py always sets pose=(0,0)=waypoints[0] at
    # start). Caught a real bug this way on real hardware: the lookahead
    # point came back as the FINAL waypoint instead of a nearby point,
    # because _advance_nearest_index() advances past waypoint 0
    # immediately (distance 0 < wp_thresh), and the old search started
    # exactly at the cursor, skipping the one segment that actually
    # mattered. This check would have caught it before the hardware did.
    pp2 = PurePursuit(path)
    pose2 = Pose(x=0.0, y=0.0, yaw=0.0)
    steer_deg, speed = pp2.update(pose2)
    lx, ly = pp2.last_lookahead
    ok = lx < 5.0  # should be ~0.8 (Ld), NOT 10.0 (the final waypoint)
    print(f"Check 2 (exact-start real-mission case): lookahead=({lx:.2f}, "
          f"{ly:.2f}) steer={steer_deg:.1f} -- "
          f"{'OK' if ok else 'FAIL: lookahead jumped to the path end!'}")

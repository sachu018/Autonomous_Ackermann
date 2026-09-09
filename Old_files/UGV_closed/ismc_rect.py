# ismc_rect.py
# Integral Sliding Mode Controller (ISMC) for Rectangle Path with Corner Stop & Turn-In-Place.
# Uses exact ISMC formulation from ismc_controller.py on straight segments with tapered corner turning.

import math
import time
from config import V_MAX, W_MAX, KP_LIN

def wrap_to_pi(angle: float) -> float:
    """Wraps an angle to the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

class ISMCControllerRect:
    def __init__(self):
        # Exact ISMC gains matching ismc_controller.py
        self.c1 = 0.45          # Cross-track error weight
        self.c2 = 0.12          # Integral state weight
        self.ks = 0.35          # Proportional reaching gain for straight paths
        self.ks_turn = 0.70     # Reaching gain for corner in-place turns
        self.eta = 0.02         # Discontinuous reaching gain
        self.phi = 0.60         # Boundary layer thickness
        self.max_lin = V_MAX    # Max linear speed (0.366 m/s)
        self.max_ang = W_MAX    # Max angular velocity (1.22 rad/s)
        
        # Corner turn-in-place threshold angles
        self.turn_entry_angle = math.radians(25.0)  # Enter turn-in-place at corners (yaw error >= 25 deg)
        self.turn_exit_angle = math.radians(6.0)    # Exit turn-in-place when aligned (yaw error <= 6 deg)
        self.rotating_in_place = False

        self.integral_state = 0.0
        self.last_time = None

    def reset(self):
        """Resets controller integral and rotation states."""
        self.integral_state = 0.0
        self.last_time = None
        self.rotating_in_place = False

    def update(self, x: float, y: float, yaw: float, current_wp: tuple, prev_wp: tuple = None, next_wp: tuple = None) -> tuple:
        """
        Calculates linear velocity (V) and angular velocity (W).
        - On straight segments: Uses exact ISMC formulation from ismc_controller.py.
        - At corners: Stops vehicle (V = 0), rotates in place with tapered deceleration, then smoothly resumes straight tracking.
        """
        if current_wp is None or current_wp[0] == 0.0:
            return 0.0, 0.0

        # Direct waypoint fallback if prev_wp is missing
        if prev_wp is None:
            dx = current_wp[0] - x
            dy = current_wp[1] - y
            dist = math.hypot(dx, dy)
            target_yaw = math.atan2(dy, dx)
            yaw_err = wrap_to_pi(target_yaw - yaw)
            
            if abs(yaw_err) > self.turn_entry_angle:
                return 0.0, max(-self.max_ang, min(self.max_ang, self.ks_turn * yaw_err))
                
            cmd_v = self.max_lin * max(0.2, min(1.0, dist / 2.0))
            cmd_w = 0.5 * yaw_err
            return cmd_v, max(-self.max_ang, min(self.max_ang, cmd_w))

        # 1. Extract segment coordinates
        x0, y0 = prev_wp[0], prev_wp[1]
        x1, y1 = current_wp[0], current_wp[1]
        
        dx_seg = x1 - x0
        dy_seg = y1 - y0
        seg_len = math.hypot(dx_seg, dy_seg)
        
        if seg_len < 1e-4:
            return 0.0, 0.0
            
        path_yaw = math.atan2(dy_seg, dx_seg)
        
        dx_ugv = x - x0
        dy_ugv = y - y0
        
        # Perpendicular cross-track error (ey) and heading error (etheta)
        ey = (dx_seg * dy_ugv - dy_seg * dx_ugv) / seg_len
        etheta = wrap_to_pi(yaw - path_yaw)
        target_yaw_err = wrap_to_pi(path_yaw - yaw)
        abs_yaw_err = abs(target_yaw_err)

        # 2. Corner Turn-In-Place Hysteresis State Machine
        if abs_yaw_err >= self.turn_entry_angle:
            self.rotating_in_place = True
        elif abs_yaw_err <= self.turn_exit_angle:
            self.rotating_in_place = False

        if self.rotating_in_place:
            # CORNER ROTATE-IN-PLACE MODE: Stop vehicle (V = 0), reset integral state, turn in place with tapered deceleration
            self.integral_state = 0.0
            self.last_time = time.monotonic()
            V = 0.0
            
            w_raw = self.ks_turn * target_yaw_err
            abs_w = abs(w_raw)
            if abs_w > 1e-3:
                # Dynamically taper min_turn from 0.48 rad/s (high torque start) down to 0.26 rad/s (gentle finish)
                # to prevent rotational overshooting at corner exit
                taper_ratio = max(0.0, min(1.0, (abs_yaw_err - self.turn_exit_angle) / (self.turn_entry_angle - self.turn_exit_angle)))
                min_turn = 0.26 + 0.22 * taper_ratio
                
                abs_w = max(min_turn, min(self.max_ang, abs_w))
                W = math.copysign(abs_w, target_yaw_err)
            else:
                W = 0.0
            return V, W

        # 3. STRAIGHT SEGMENT ISMC CONTROL MODE (Exact formulation as ismc_controller.py)
        now = time.monotonic()
        dt = 0.05 if self.last_time is None else (now - self.last_time)
        self.last_time = now
        
        # Update Integral State with anti-windup clamping
        term = etheta + self.c1 * ey
        self.integral_state += term * dt
        self.integral_state = max(-1.0, min(1.0, self.integral_state))

        # Sliding Surface: s = e_theta + c1 * e_y + c2 * integral
        s = etheta + self.c1 * ey + self.c2 * self.integral_state
        sat_s = s / (abs(s) + self.phi)

        # Target Speed calculation with smooth cosine-squared corner launch scaling
        dist_to_wp = math.hypot(x1 - x, y1 - y)
        cmd_v = min(KP_LIN * dist_to_wp, self.max_lin)
        cmd_v = max(0.05, cmd_v)
        
        speed_scale = max(0.20, math.cos(etheta) ** 2)
        cmd_v *= speed_scale

        # ISMC Control Law: Equivalent control + Reaching control
        v_steer = max(0.20, cmd_v)
        equivalent_control = -self.c1 * v_steer * math.sin(etheta) - self.c2 * (etheta + self.c1 * ey)
        reaching_control = -self.ks * s - self.eta * sat_s
        
        cmd_w = equivalent_control + reaching_control
        cmd_w = max(-self.max_ang, min(self.max_ang, cmd_w))

        return cmd_v, cmd_w




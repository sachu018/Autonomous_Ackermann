# p_controller_rect.py
# High-Precision Waypoint tracking P-Controller for Rectangle Path with Tapered Corner Turn-In-Place.

import math
from config import (
    KP_LIN, KP_ANG, V_MAX, W_MAX, YAW_BOUND_DEG, LOOK_AHEAD_DIST
)

def wrap_to_pi(angle: float) -> float:
    """Wraps an angle to the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

class WaypointControllerRect:
    def __init__(self, look_ahead_dist: float = LOOK_AHEAD_DIST):
        self.Kp_lin = KP_LIN
        self.Kp_ang = 0.52             # Slightly increased angular gain for crisp line tracking
        self.Kp_ang_turn = 0.70        # Proportional gain for corner turn-in-place
        self.max_lin = V_MAX           # Max linear speed (0.366 m/s)
        self.max_ang = W_MAX           # Max angular velocity (1.22 rad/s)
        self.yaw_bound = math.radians(YAW_BOUND_DEG)
        self.look_ahead_dist = 0.75    # Slightly tighter look-ahead distance (0.75m) for sub-2.5cm CTE
        
        # Corner turn-in-place threshold angles
        self.turn_entry_angle = math.radians(25.0)  # Enter turn-in-place at corners (yaw error >= 25 deg)
        self.turn_exit_angle = math.radians(5.5)    # Exit turn-in-place when aligned (yaw error <= 5.5 deg)
        self.rotating_in_place = False

    def reset(self):
        """Resets controller state."""
        self.rotating_in_place = False

    def update(self, x: float, y: float, yaw: float, current_wp: tuple, prev_wp: tuple = None, next_wp: tuple = None) -> tuple:
        """
        Calculates linear velocity (V) and angular velocity (W).
        - On straight segments: Uses standard P-controller (p_controller.py) with tuned 0.75m look-ahead.
        - At corners: Stops vehicle (V = 0), rotates in place with tapered deceleration to prevent overshooting,
          then smoothly transitions back to straight line tracking.
        """
        if current_wp is None or current_wp[0] == 0.0:
            return 0.0, 0.0

        dist_to_goal = math.hypot(current_wp[0] - x, current_wp[1] - y)
        
        # 1. Target Yaw Calculation using Look-Ahead on current path segment
        if prev_wp is not None:
            dx_seg = current_wp[0] - prev_wp[0]
            dy_seg = current_wp[1] - prev_wp[1]
            seg_len = math.hypot(dx_seg, dy_seg)
            
            if seg_len > 1e-4:
                # Project UGV position onto segment
                t = ((x - prev_wp[0]) * dx_seg + (y - prev_wp[1]) * dy_seg) / (seg_len ** 2)
                t = max(0.0, min(1.0, t))
                
                x_close = prev_wp[0] + t * dx_seg
                y_close = prev_wp[1] + t * dy_seg
                
                # Target point is look_ahead_dist ahead along path vector
                x_la = x_close + self.look_ahead_dist * (dx_seg / seg_len)
                y_la = y_close + self.look_ahead_dist * (dy_seg / seg_len)
                target_yaw = math.atan2(y_la - y, x_la - x)
            else:
                target_yaw = math.atan2(current_wp[1] - y, current_wp[0] - x)
        else:
            target_yaw = math.atan2(current_wp[1] - y, current_wp[0] - x)

        # 2. Yaw Error calculation
        yaw_err = wrap_to_pi(target_yaw - yaw)
        abs_yaw_err = abs(yaw_err)

        # 3. Corner Turn-In-Place Hysteresis State Machine
        if abs_yaw_err >= self.turn_entry_angle:
            self.rotating_in_place = True
        elif abs_yaw_err <= self.turn_exit_angle:
            self.rotating_in_place = False

        if self.rotating_in_place:
            # CORNER ROTATE-IN-PLACE MODE: Stop vehicle (V = 0), turn in place with tapered deceleration
            V = 0.0
            w_raw = self.Kp_ang_turn * yaw_err
            abs_w = abs(w_raw)
            if abs_w > 1e-3:
                # Dynamically taper min_turn from 0.48 rad/s (high torque start) down to 0.26 rad/s (gentle finish)
                # to eliminate rotational overshooting at corner exit
                taper_ratio = max(0.0, min(1.0, (abs_yaw_err - self.turn_exit_angle) / (self.turn_entry_angle - self.turn_exit_angle)))
                min_turn = 0.26 + 0.22 * taper_ratio
                
                abs_w = max(min_turn, min(self.max_ang, abs_w))
                W = math.copysign(abs_w, yaw_err)
            else:
                W = 0.0
            return V, W

        # 4. STRAIGHT LINE SEGMENT MODE (Exact formulation as p_controller.py)
        # Limit yaw error calculation to bound steering authority (eliminates aggressive weaves)
        yaw_err_bounded = max(-self.yaw_bound, min(self.yaw_bound, yaw_err))

        # Speed Controller: Scale speed down proportionally during heading corrections
        V = min(self.Kp_lin * dist_to_goal, self.max_lin)
        V = max(0.05, V)  # Enforce a slow crawl minimum speed to prevent stalling
        
        # Smooth cosine-squared attenuation for soft launch out of corners
        speed_scale = max(0.20, math.cos(yaw_err_bounded) ** 2)
        V *= speed_scale

        # Steering (Angular Speed) Controller
        W = self.Kp_ang * yaw_err_bounded
        W = max(-self.max_ang, min(self.max_ang, W))

        return V, W





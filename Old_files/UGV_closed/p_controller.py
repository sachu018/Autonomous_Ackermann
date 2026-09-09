# p_controller.py
# Waypoint tracking guidance controller using proportional look-ahead feedback.

import math
from config import (
    KP_LIN, KP_ANG, V_MAX, W_MAX,
    YAW_METHOD, LOOK_AHEAD_DIST, YAW_BOUND_DEG
)

def wrap_to_pi(angle: float) -> float:
    """Wraps an angle to the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

class WaypointController:
    def __init__(self):
        self.Kp_lin = KP_LIN
        self.Kp_ang = KP_ANG
        self.max_lin = V_MAX
        self.max_ang = W_MAX
        self.yaw_bound = math.radians(YAW_BOUND_DEG)

    def update(self, x: float, y: float, yaw: float, current_wp: tuple, prev_wp: tuple = None) -> tuple:
        """
        Calculates linear velocity (V) and angular velocity (W).
        Args:
            x, y, yaw: Current UGV pose
            current_wp: Target waypoint coordinates (X, Y)
            prev_wp: Previous waypoint coordinates (X, Y) (optional, for path projection)
        Returns:
            V (float): Desired linear velocity (m/s)
            W (float): Desired angular velocity (rad/s)
        """
        dist_to_goal = math.hypot(current_wp[0] - x, current_wp[1] - y)
        
        # 1. Target Yaw Calculation
        if YAW_METHOD == "LOOK_AHEAD" and prev_wp is not None:
            dx_seg = current_wp[0] - prev_wp[0]
            dy_seg = current_wp[1] - prev_wp[1]
            seg_len = math.hypot(dx_seg, dy_seg)
            
            if seg_len > 1e-4:
                # Project UGV position onto the line segment to find the closest point
                t = ((x - prev_wp[0]) * dx_seg + (y - prev_wp[1]) * dy_seg) / (seg_len ** 2)
                t = max(0.0, min(1.0, t))  # Clamp to segment boundaries
                
                x_close = prev_wp[0] + t * dx_seg
                y_close = prev_wp[1] + t * dy_seg
                
                # Place look-ahead target L_d ahead along the segment path vector
                x_la = x_close + LOOK_AHEAD_DIST * (dx_seg / seg_len)
                y_la = y_close + LOOK_AHEAD_DIST * (dy_seg / seg_len)
            else:
                x_la, y_la = current_wp[0], current_wp[1]
                
            target_yaw = math.atan2(y_la - y, x_la - x)
        else:
            # DIRECT_WAYPOINT method (pointing directly to next target coordinate)
            target_yaw = math.atan2(current_wp[1] - y, current_wp[0] - x)

        # 2. Yaw Error calculation
        yaw_err = wrap_to_pi(target_yaw - yaw)
        
        # Limit yaw error calculation to bound steering authority (eliminates aggressive weaves)
        yaw_err_bounded = max(-self.yaw_bound, min(self.yaw_bound, yaw_err))

        # 3. Speed Controller
        # Linear velocity is proportional to distance, capped at max_lin
        V = min(self.Kp_lin * dist_to_goal, self.max_lin)
        V = max(0.05, V)  # Enforce a slow crawl minimum speed to prevent stalling
        
        # Scale speed down proportionally based on misalignment (slow down in sharp curves)
        V *= max(0.2, math.cos(yaw_err_bounded))

        # 4. Steering (Angular Speed) Controller
        # Angular velocity is proportional to yaw error, capped at max_ang
        W = self.Kp_ang * yaw_err_bounded
        W = max(-self.max_ang, min(self.max_ang, W))

        return V, W

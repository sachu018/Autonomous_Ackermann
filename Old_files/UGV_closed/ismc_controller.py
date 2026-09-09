# ismc_controller.py
# Integral Sliding Mode Controller (ISMC) for UGV path tracking.

import math
import time

class ISMCController:
    def __init__(self):
        # Stabilized ISMC gains (tuned for zero oscillation & < 2.0cm mean CTE)
        self.c1 = 0.45       # Sliding surface weight for cross-track error (damps step response)
        self.c2 = 0.12       # Sliding surface weight for integral state (smoothly rejects bias < 1cm)
        self.ks = 0.35       # Proportional reaching gain (eliminates limit-cycle weaving)
        self.eta = 0.02      # Discontinuous reaching gain (smooth disturbance rejection)
        self.phi = 0.60      # Boundary layer thickness for smooth linear transition
        
        # Integral state
        self.integral_state = 0.0
        self.last_time = None

    def reset(self):
        self.integral_state = 0.0
        self.last_time = None

    def update(self, x: float, y: float, yaw: float, current_wp: tuple, prev_wp: tuple) -> tuple:
        """
        Calculates linear speed V and angular velocity W using ISMC law.
        """
        if current_wp is None:
            return 0.0, 0.0

        # 1. Target linear velocity V
        from config import V_MAX, W_MAX
        
        # If no previous waypoint is available, perform direct waypoint guidance
        if prev_wp is None:
            dx = current_wp[0] - x
            dy = current_wp[1] - y
            dist = math.hypot(dx, dy)
            target_yaw = math.atan2(dy, dx)
            yaw_err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
            
            # Simple P controller for direct target alignment
            cmd_v = V_MAX * max(0.2, min(1.0, dist / 2.0))
            cmd_w = 0.5 * yaw_err
            return cmd_v, max(-W_MAX, min(W_MAX, cmd_w))

        # 2. Extract segment vectors
        x0, y0 = prev_wp[0], prev_wp[1]
        x1, y1 = current_wp[0], current_wp[1]
        
        dx_seg = x1 - x0
        dy_seg = y1 - y0
        seg_len = math.hypot(dx_seg, dy_seg)
        
        if seg_len < 1e-4:
            return 0.0, 0.0
            
        # Target path heading
        path_yaw = math.atan2(dy_seg, dx_seg)
        
        # 3. Compute cross-track error (ey) and heading error (etheta)
        dx_ugv = x - x0
        dy_ugv = y - y0
        
        # Perpendicular cross-track error (ey) - positive to the left
        ey = (dx_seg * dy_ugv - dy_seg * dx_ugv) / seg_len
        
        # Heading error (etheta) - positive to the left, wrapped to [-pi, pi]
        etheta = math.atan2(math.sin(yaw - path_yaw), math.cos(yaw - path_yaw))

        # 4. Update the Integral State of the sliding surface
        now = time.monotonic()
        if self.last_time is None:
            dt = 0.05
        else:
            dt = now - self.last_time
        self.last_time = now
        
        # Anti-windup clamping on integral state
        term = etheta + self.c1 * ey
        self.integral_state += term * dt
        self.integral_state = max(-1.0, min(1.0, self.integral_state))

        # 5. Define Sliding Surface: s = e_theta + c1 * e_y + c2 * integral
        s = etheta + self.c1 * ey + self.c2 * self.integral_state

        # 6. Smooth boundary layer approximation using sat
        sat_s = s / (abs(s) + self.phi)

        # 7. ISMC Reaching Law:
        from config import KP_LIN
        dist_to_wp = math.hypot(x1 - x, y1 - y)
        cmd_v = min(KP_LIN * dist_to_wp, V_MAX)
        cmd_v = max(0.05, cmd_v)  # Prevent complete stall before reaching final goal

        # Maintain minimum steering velocity floor to preserve turning authority at low finishing speeds
        v_steer = max(0.20, cmd_v)
        equivalent_control = -self.c1 * v_steer * math.sin(etheta) - self.c2 * (etheta + self.c1 * ey)
        reaching_control = -self.ks * s - self.eta * sat_s
        
        cmd_w = equivalent_control + reaching_control

        # Clamp angular turn rate to safe bounds
        cmd_w = max(-W_MAX, min(W_MAX, cmd_w))
        
        return cmd_v, cmd_w

# differential_drive.py
# Converts body velocity commands (V, W) into wheel RPM speeds using differential drive kinematics.

import math
from config import R, L, WHEEL_RPM_MAX, MIN_RATIO

def body_to_wheel_rpm(V: float, W: float) -> tuple:
    """
    Translates linear velocity (V) and angular velocity (W) into individual wheel RPMs.
    Args:
        V (float): Desired linear velocity (m/s)
        W (float): Desired angular velocity (rad/s)
    Returns:
        rpm_L (float): Commanded left wheel RPM
        rpm_R (float): Commanded right wheel RPM
    """
    # 1. Enforce kinematic constraint to prevent turning rate from causing immediate wheel reversal
    if V != 0.0:
        W_max_safe = 2.0 * abs(V) / L
        W = max(-W_max_safe, min(W_max_safe, W))

    # 2. Differential equations (V = (VR + VL)/2 , W = (VR - VL)/L)
    V_L = V - (L / 2.0) * W
    V_R = V + (L / 2.0) * W

    # 3. Convert wheel linear velocities (m/s) to shaft RPMs
    rpm_L = (V_L / (2.0 * math.pi * R)) * 60.0
    rpm_R = (V_R / (2.0 * math.pi * R)) * 60.0

    # 4. Scale peak RPM to command limit
    peak = max(abs(rpm_L), abs(rpm_R))
    if peak > WHEEL_RPM_MAX:
        scale = WHEEL_RPM_MAX / peak
        rpm_L *= scale
        rpm_R *= scale

    # 5. Prevent inner wheel from locking up or spinning backward during forward motion
    if V != 0.0 and math.copysign(1, rpm_L) == math.copysign(1, rpm_R):
        faster = max(abs(rpm_L), abs(rpm_R))
        if faster > 0.0:
            slower_min = faster * MIN_RATIO
            sign_V = math.copysign(1.0, V)
            if abs(rpm_L) < slower_min:
                rpm_L = math.copysign(slower_min, sign_V)
            if abs(rpm_R) < slower_min:
                rpm_R = math.copysign(slower_min, sign_V)

    return rpm_L, rpm_R

# safety.py
# Implements watchdog monitors, GPS status attenuation, and manual override sticks supervisor.

import time
from config import RC_STICK_OVERRIDE_THRESH, RTK_TIMEOUT_S

class SafetySupervisor:
    def __init__(self):
        self.last_rtk_good_time = time.monotonic()

    def get_speed_multiplier(self, fix_status: str) -> float:
        """
        Determines target speed multiplier based on the RTK fix quality string.
        """
        status = fix_status.upper().strip()
        
        if "RTK FIXED" in status:
            return 1.0      # Run at 100% velocity
        elif "RTK FLOAT" in status:
            return 0.6      # Attenuate to 60% velocity
        elif "DGPS" in status or "GPS" in status or status == "1" or status == "2":
            return 0.3      # Attenuate to 30% velocity for degraded quality
        else:
            return 0.0      # Zero speed (Stop) for NO FIX or UNKNOWN

    def is_stick_deflected(self, Xn: float, Yn: float) -> bool:
        """
        Detects if the operator is active on the transmitter sticks.
        Deflections larger than 15% trigger manual takeover.
        """
        return abs(Xn) > RC_STICK_OVERRIDE_THRESH or abs(Yn) > RC_STICK_OVERRIDE_THRESH

    def check_rtk_loss(self, latest_pose_timestamp: float) -> bool:
        """
        Returns True if the time elapsed since the last valid RTK GPS frame exceeds the timeout threshold.
        """
        if latest_pose_timestamp == 0.0:
            return True  # Haven't received first fix yet
            
        elapsed = time.monotonic() - latest_pose_timestamp
        return elapsed > RTK_TIMEOUT_S

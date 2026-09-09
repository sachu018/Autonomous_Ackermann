# localization.py
# Handles conversion of Lat/Lon to WGS-84 UTM, spike filtering, and kinematic heading updates.

import math
import time
from config import MIN_MOVE_DIST, COORDINATE_SMOOTHING_ALPHA, RTK_OFFSET_X, RTK_OFFSET_Y

# WGS-84 Ellipsoid Constants
WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563

def latlon_to_utm(lat: float, lon: float, force_zone=43):
    """
    Converts latitude and longitude coordinates to UTM Easting (X), Northing (Y), and Zone.
    Defaults to Zone 43 (Palakkad, Kerala, India) to prevent coordinate jumps across zones.
    """
    b = WGS84_A * (1 - WGS84_F)
    e2 = (WGS84_A**2 - b**2) / WGS84_A**2
    e_prime2 = (WGS84_A**2 - b**2) / b**2
    
    if force_zone is not None:
        zone = force_zone
    else:
        zone = int((lon + 180) / 6) + 1
        
    lon_origin = (zone - 1) * 6 - 180 + 3
    lon_origin_rad = math.radians(lon_origin)
    
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    
    k0 = 0.9996  # UTM scale factor
    
    N = WGS84_A / math.sqrt(1 - e2 * math.sin(lat_rad)**2)
    T = math.tan(lat_rad)**2
    C = e_prime2 * math.cos(lat_rad)**2
    A = (lon_rad - lon_origin_rad) * math.cos(lat_rad)
    
    # Meridional Arc Length
    M = WGS84_A * (
        (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256) * lat_rad
        - (3 * e2 / 8 + 3 * e2**2 / 32 + 45 * e2**3 / 1024) * math.sin(2 * lat_rad)
        + (15 * e2**2 / 256 + 45 * e2**3 / 1024) * math.sin(4 * lat_rad)
        - (35 * e2**3 / 3072) * math.sin(6 * lat_rad)
    )
    
    easting = k0 * N * (
        A
        + (1 - T + C) * A**3 / 6
        + (5 - 18 * T + T**2 + 72 * C - 58 * e_prime2) * A**5 / 120
    ) + 500000.0
    
    northing = k0 * (
        M
        + N
        * math.tan(lat_rad)
        * (
            A**2 / 2
            + (5 - T + 9 * C + 4 * C**2) * A**4 / 24
            + (61 - 58 * T + T**2 + 600 * C - 330 * e_prime2) * A**6 / 720
        )
    )
    
    if lat < 0:
        northing += 10000000.0  # False northing for southern hemisphere
        
    return easting, northing, zone

class Localization:
    def __init__(self):
        self.x = None            # Current Easting (meters)
        self.y = None            # Current Northing (meters)
        self.zone = None         # UTM Zone
        self.yaw = 0.0           # Current yaw heading in radians (-pi to pi)
        
        self.prev_x = None       # Reference X for heading estimation
        self.prev_y = None       # Reference Y for heading estimation
        
        self.raw_x = None        # Raw Easting (for spikes detection and logger pass-through)
        self.raw_y = None        # Raw Northing (for spikes detection and logger pass-through)
        self.alpha = COORDINATE_SMOOTHING_ALPHA  # Coordinate smoothing factor (same as logger)
        
        self.last_update_time = None
        self.max_speed_limit = 5.0  # GPS spike filter: displacements yielding > 5 m/s are flagged

    def update(self, lat: float, lon: float, rtk_heading_deg: float = 0.0, current_time: float = None) -> bool:
        """
        Updates the pose estimate using new RTK coordinates.
        Returns:
            bool: True if coordinate update was accepted, False if flagged as GPS spike outlier.
        """
        if current_time is None:
            current_time = time.time()
            
        easting, northing, zone = latlon_to_utm(lat, lon)
        
        # Project raw GPS antenna coordinate to the UGV center of rotation
        # Note: self.yaw is counter-clockwise from East (Cartesian space)
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        easting_center = easting - RTK_OFFSET_X * cos_yaw + RTK_OFFSET_Y * sin_yaw
        northing_center = northing - RTK_OFFSET_X * sin_yaw - RTK_OFFSET_Y * cos_yaw
        
        # 1. Initialize first position
        if self.x is None:
            self.x = easting_center
            self.y = northing_center
            self.raw_x = easting_center
            self.raw_y = northing_center
            self.zone = zone
            self.prev_x = easting_center
            self.prev_y = northing_center
            self.last_update_time = current_time
            
            # Initialize heading from GPS receiver if provided, otherwise preserve preset yaw (path heading)
            if rtk_heading_deg > 0.0:
                # Convert compass heading (clockwise from North) to Cartesian (counterclockwise from East)
                self.yaw = math.radians((90.0 - rtk_heading_deg) % 360.0)
                
            return True

        # 2. GPS Spike / Outlier Filter (based on raw position drift)
        dt = current_time - self.last_update_time
        if dt <= 0:
            dt = 0.01
            
        dx = easting_center - self.raw_x
        dy = northing_center - self.raw_y
        dist = math.hypot(dx, dy)
        speed = dist / dt
        
        if speed > self.max_speed_limit:
            # Flagged as spike - discard reading, hold last position
            print(f"[Localization] Warning: Discarded GPS spike (calculated speed: {speed:.2f} m/s)")
            return False
            
        # Update raw state
        self.raw_x = easting_center
        self.raw_y = northing_center

        # Apply online EMA coordinate smoothing
        self.x = self.alpha * easting_center + (1.0 - self.alpha) * self.x
        self.y = self.alpha * northing_center + (1.0 - self.alpha) * self.y

        # 3. Kinematic Heading Update (with Jitter Guard, using smoothed coordinates)
        dx_move = self.x - self.prev_x
        dy_move = self.y - self.prev_y
        dist_moved = math.hypot(dx_move, dy_move)
        
        # Require 8cm of true vehicle displacement to update heading (filters standing GPS noise)
        if dist_moved >= 0.08:
            # Kinematic motion heading
            new_yaw = math.atan2(dy_move, dx_move)
            
            # Smooth yaw using wrapping calculation (30% weight to new heading, 70% historic)
            yaw_diff = math.atan2(math.sin(new_yaw - self.yaw), math.cos(new_yaw - self.yaw))
            self.yaw = math.atan2(math.sin(self.yaw + 0.3 * yaw_diff), math.cos(self.yaw + 0.3 * yaw_diff))
            
            self.prev_x = self.x
            self.prev_y = self.y
            
        self.last_update_time = current_time
        return True

    def update_in_place_yaw(self, cmd_w: float, dt: float = 0.05):
        """Integrates commanded angular velocity to update yaw during in-place rotation (V=0)."""
        if abs(cmd_w) > 0.01:
            self.yaw = math.atan2(math.sin(self.yaw + cmd_w * dt), math.cos(self.yaw + cmd_w * dt))

    def get_pose(self):
        """Returns the current pose tuple (X, Y, Yaw, Zone)"""
        return self.x, self.y, self.yaw, self.zone

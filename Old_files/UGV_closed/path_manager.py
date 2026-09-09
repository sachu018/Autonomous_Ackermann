# path_manager.py
# Loads waypoint paths from CSV files and coordinates waypoint switching.

import csv
import math
import sys
from config import WP_THRESH

class PathManager:
    def __init__(self, csv_path: str):
        self.csv_path = csv_path
        self.waypoints = []  # List of tuples: (X, Y)
        self.current_idx = 0  # Start at the first waypoint (index 0)
        self.load_waypoints()

    def load_waypoints(self):
        """Loads waypoints from the given CSV path. Automatically maps UTM and GPS headers."""
        try:
            with open(self.csv_path, 'r') as f:
                reader = csv.reader(f)
                headers = [h.strip().lower() for h in next(reader)]
                
                # Check for standard headers (prioritizing smoothed columns)
                x_idx, y_idx = None, None
                for key in ['smoothed_utm_x', 'smoothed_utm_easting', 'utm_x', 'x', 'easting']:
                    if key in headers:
                        x_idx = headers.index(key)
                        break
                for key in ['smoothed_utm_y', 'smoothed_utm_northing', 'utm_y', 'y', 'northing']:
                    if key in headers:
                        y_idx = headers.index(key)
                        break
                
                # Check for GPS coordinates (to log targets)
                lat_idx, lon_idx = None, None
                for key in ['smoothed_latitude', 'smoothed_lat', 'latitude', 'lat']:
                    if key in headers:
                        lat_idx = headers.index(key)
                        break
                for key in ['smoothed_longitude', 'smoothed_lon', 'longitude', 'lon', 'lng']:
                    if key in headers:
                        lon_idx = headers.index(key)
                        break
                
                # Positional fallback if headers are not found
                if x_idx is None or y_idx is None:
                    print("[PathManager] Warning: Coordinate headers not matched. Using fallback columns 1 and 2.")
                    x_idx = 1
                    y_idx = 2
                    
                for row in reader:
                    if len(row) > max(x_idx, y_idx):
                        try:
                            x_val = float(row[x_idx])
                            y_val = float(row[y_idx])
                            lat_val = float(row[lat_idx]) if (lat_idx is not None and len(row) > lat_idx) else 0.0
                            lon_val = float(row[lon_idx]) if (lon_idx is not None and len(row) > lon_idx) else 0.0
                            self.waypoints.append((x_val, y_val, lat_val, lon_val))
                        except ValueError:
                            continue  # Skip row on parse failure
                            
            print(f"[PathManager] Loaded {len(self.waypoints)} waypoints from {self.csv_path}")
        except Exception as e:
            print(f"[PathManager] Error loading path file: {e}")
            sys.exit(1)

    def get_current_waypoint(self):
        """Returns the current target waypoint tuple (X, Y)."""
        if self.current_idx < len(self.waypoints):
            return self.waypoints[self.current_idx]
        return None

    def get_previous_waypoint(self):
        """Returns the previous waypoint tuple (X, Y) which defines the current path segment."""
        if self.current_idx > 0 and (self.current_idx - 1) < len(self.waypoints):
            return self.waypoints[self.current_idx - 1]
        return None

    def get_next_waypoint(self):
        """Returns the next waypoint tuple (X, Y) after current_idx."""
        if (self.current_idx + 1) < len(self.waypoints):
            return self.waypoints[self.current_idx + 1]
        return None

    def get_all_waypoints(self):
        return self.waypoints

    def update(self, x: float, y: float) -> bool:
        """
        Calculates distance to target waypoint and switches index if within threshold.
        Returns:
            bool: True if waypoint index was advanced, False otherwise.
        """
        target = self.get_current_waypoint()
        if target is None:
            return False
            
        dist = math.hypot(target[0] - x, target[1] - y)
        if dist < WP_THRESH:
            print(f"\n[PathManager] Reached Waypoint {self.current_idx}/{len(self.waypoints)-1}")
            self.current_idx += 1
            return True

        # Segment progress projection check (prevent overshoot stuck loops)
        if self.current_idx > 0:
            prev_wp = self.waypoints[self.current_idx - 1]
            dx_seg = target[0] - prev_wp[0]
            dy_seg = target[1] - prev_wp[1]
            seg_len = math.hypot(dx_seg, dy_seg)

            if seg_len > 1e-4:
                t = ((x - prev_wp[0]) * dx_seg + (y - prev_wp[1]) * dy_seg) / (seg_len ** 2)
                if t >= 0.95:
                    print(f"\n[PathManager] Passed Waypoint {self.current_idx}/{len(self.waypoints)-1} segment line, advancing.")
                    self.current_idx += 1
                    return True
            
        return False

    def is_mission_finished(self) -> bool:
        """Returns True if the UGV has completed all waypoints."""
        return self.current_idx >= len(self.waypoints)

    def align_start_to_actual_position(self, x: float, y: float):
        """Sets Waypoint 0 to the UGV's actual starting position to eliminate initial tracking errors."""
        if not self.waypoints:
            return
            
        # Update Waypoint 0 to the exact physical starting position of the UGV
        lat0 = self.waypoints[0][2] if len(self.waypoints[0]) > 2 else 0.0
        lon0 = self.waypoints[0][3] if len(self.waypoints[0]) > 3 else 0.0
        self.waypoints[0] = (x, y, lat0, lon0)
        self.current_idx = 1
        print(f"[PathManager] Startup Calibration: Set Waypoint 0 to match UGV starting position X:{x:.3f} Y:{y:.3f}")

    def align_to_nearest_start_segment(self, x: float, y: float):
        """Aligns current_idx to the nearest segment to eliminate initial approach transients."""
        if len(self.waypoints) < 2:
            return
            
        best_idx = 1
        min_dist = float('inf')
        
        for i in range(1, len(self.waypoints)):
            ax, ay = self.waypoints[i-1][0], self.waypoints[i-1][1]
            bx, by = self.waypoints[i][0], self.waypoints[i][1]
            dx, dy = bx - ax, by - ay
            seg_len = math.hypot(dx, dy)
            if seg_len > 1e-4:
                t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (seg_len ** 2)))
                dist = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
                if dist < min_dist:
                    min_dist = dist
                    best_idx = i
                    
        self.current_idx = best_idx
        print(f"[PathManager] Aligned starting segment to Waypoint {self.current_idx} (Distance to line: {min_dist*100:.1f} cm)")

    def reset(self):
        self.current_idx = 0

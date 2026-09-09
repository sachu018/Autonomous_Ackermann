#!/usr/bin/env python3
# generate_straight_line.py
# Helper utility to generate a straight-line waypoint CSV file from two endpoint coordinates.

import sys
import os
import csv
import math

# Import UTM conversion from localization module
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from localization import latlon_to_utm

def generate_straight_line(lat1: float, lon1: float, lat2: float, lon2: float, 
                           spacing: float = 0.5, speed: float = 0.15, output_file: str = "waypoints_1m.csv"):
    """
    Generates a straight path CSV from two Latitude/Longitude endpoints.
    Interpolates waypoints at a fixed spacing distance in meters.
    """
    # 1. Convert start/end coordinates to UTM Zone 43
    x1, y1, zone1 = latlon_to_utm(lat1, lon1)
    x2, y2, zone2 = latlon_to_utm(lat2, lon2)
    
    print(f"[Generator] Start position: UTM X={x1:.3f}, Y={y1:.3f} (Zone {zone1})")
    print(f"[Generator] End position:   UTM X={x2:.3f}, Y={y2:.3f} (Zone {zone2})")
    
    # 2. Calculate segment properties
    dx = x2 - x1
    dy = y2 - y1
    total_distance = math.hypot(dx, dy)
    heading_rad = math.atan2(dy, dx)
    heading_deg = math.degrees(heading_rad) % 360.0
    
    print(f"[Generator] Path length: {total_distance:.2f} meters")
    print(f"[Generator] Kinematic heading: {heading_deg:.1f}°")
    
    num_points = int(total_distance / spacing) + 1
    
    # 3. Interpolate points
    waypoints = []
    for i in range(num_points):
        t = i / (num_points - 1) if num_points > 1 else 0.0
        lat = lat1 + t * (lat2 - lat1)
        lon = lon1 + t * (lon2 - lon1)
        x, y, zone = latlon_to_utm(lat, lon)
        dist_from_start = t * total_distance
        
        waypoints.append({
            "Index": i,
            "Smoothed_UTM_X": round(x, 3),
            "Smoothed_UTM_Y": round(y, 3),
            "Latitude": round(lat, 8),
            "Longitude": round(lon, 8),
            "Calculated_Yaw_Deg": round(heading_deg, 1),
            "Distance": round(dist_from_start, 2),
            "Speed": speed
        })
        
    # 4. Export to CSV
    import config
    output_path = os.path.join(config.BASE_DIR, output_file)
    
    try:
        with open(output_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Index", "Smoothed_UTM_X", "Smoothed_UTM_Y", "Latitude", "Longitude", "Calculated_Yaw_Deg", "Distance", "Speed"])
            for wp in waypoints:
                writer.writerow([
                    wp["Index"],
                    wp["Smoothed_UTM_X"],
                    wp["Smoothed_UTM_Y"],
                    wp["Latitude"],
                    wp["Longitude"],
                    wp["Calculated_Yaw_Deg"],
                    wp["Distance"],
                    wp["Speed"]
                ])
        print(f"✅ Generated {len(waypoints)} waypoints. Saved to: {output_path}")
    except Exception as e:
        print(f"Error writing waypoints file: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 5:
        print("Usage: python3 generate_straight_line.py <lat1> <lon1> <lat2> <lon2> [spacing_m] [speed_m_s] [output_file.csv]")
        print("Example: python3 generate_straight_line.py 10.791234 76.241234 10.791567 76.241567 0.5 0.15 waypoints_1m.csv")
        sys.exit(1)
        
    lat_start = float(sys.argv[1])
    lon_start = float(sys.argv[2])
    lat_end = float(sys.argv[3])
    lon_end = float(sys.argv[4])
    
    space = float(sys.argv[5]) if len(sys.argv) > 5 else 0.5
    spd = float(sys.argv[6]) if len(sys.argv) > 6 else 0.15
    out = sys.argv[7] if len(sys.argv) > 7 else "waypoints_1m.csv"
    
    generate_straight_line(lat_start, lon_start, lat_end, lon_end, space, spd, out)

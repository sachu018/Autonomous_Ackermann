#!/usr/bin/env python3
# generate_rectangle.py
# Helper utility to generate rectangle corner coordinates and dense interpolated path waypoints.

import sys
import os
import csv
import math

# Import UTM conversion from localization module
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from localization import latlon_to_utm

def interpolate_segment_latlon(lat1, lon1, lat2, lon2, spacing=1.0):
    """Interpolates points along a single straight line segment in Lat/Lon and UTM."""
    x1, y1, _ = latlon_to_utm(lat1, lon1)
    x2, y2, _ = latlon_to_utm(lat2, lon2)
    dx = x2 - x1
    dy = y2 - y1
    segment_len = math.hypot(dx, dy)
    heading_deg = math.degrees(math.atan2(dy, dx)) % 360.0
    
    num_points = int(segment_len / spacing)
    points = []
    
    for i in range(num_points):
        t = i / num_points if num_points > 0 else 0.0
        lat = lat1 + t * (lat2 - lat1)
        lon = lon1 + t * (lon2 - lon1)
        x = x1 + t * dx
        y = y1 + t * dy
        points.append((x, y, lat, lon, heading_deg))
        
    return points, segment_len

def generate_rectangle_path(lat1, lon1, lat2, lon2, lat3, lon3, lat4, lon4, 
                            spacing=1.0, speed=0.15, base_dir=None):
    """
    Generates:
      1. rectangle_corners.csv (Raw UTM coordinates of the corners)
      2. rectangle_path.csv (Interpolated path at spacing interval)
    """
    if base_dir is None:
        import config
        base_dir = config.BASE_DIR
        
    # 1. Convert all corners to UTM Zone 43
    corners = []
    for idx, (lat, lon) in enumerate([(lat1, lon1), (lat2, lon2), (lat3, lon3), (lat4, lon4)]):
        x, y, zone = latlon_to_utm(lat, lon)
        corners.append((idx + 1, x, y, lat, lon))
        
    # Write Corners File
    corners_filepath = os.path.join(base_dir, "rectangle_corners.csv")
    with open(corners_filepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Corner_ID", "UTM_X", "UTM_Y", "Latitude", "Longitude"])
        for c in corners:
            writer.writerow([c[0], f"{c[1]:.3f}", f"{c[2]:.3f}", f"{c[3]:.8f}", f"{c[4]:.8f}"])
    print(f"[Rectangle] Corners list saved to: {corners_filepath}")

    # 2. Interpolate segments (Corner 1 -> 2 -> 3 -> 4 -> 1 to close loop)
    lat_coords = [c[3] for c in corners] + [corners[0][3]]
    lon_coords = [c[4] for c in corners] + [corners[0][4]]
    x_coords = [c[1] for c in corners] + [corners[0][1]]
    y_coords = [c[2] for c in corners] + [corners[0][2]]
    
    path_points = []
    total_accumulated_dist = 0.0
    
    for s in range(4):
        lat_start, lon_start = lat_coords[s], lon_coords[s]
        lat_end, lon_end = lat_coords[s+1], lon_coords[s+1]
        
        seg_points, seg_len = interpolate_segment_latlon(lat_start, lon_start, lat_end, lon_end, spacing)
        
        for pt in seg_points:
            path_points.append({
                "Smoothed_UTM_X": pt[0],
                "Smoothed_UTM_Y": pt[1],
                "Latitude": pt[2],
                "Longitude": pt[3],
                "Calculated_Yaw_Deg": pt[4],
                "Distance": total_accumulated_dist + math.hypot(pt[0]-x_coords[s], pt[1]-y_coords[s]),
                "Speed": speed
            })
            
        total_accumulated_dist += seg_len
        
    # Append final starting corner to close loop
    path_points.append({
        "Smoothed_UTM_X": corners[0][1],
        "Smoothed_UTM_Y": corners[0][2],
        "Latitude": corners[0][3],
        "Longitude": corners[0][4],
        "Calculated_Yaw_Deg": path_points[-1]["Calculated_Yaw_Deg"],
        "Distance": total_accumulated_dist,
        "Speed": speed
    })
    
    # Write Path File
    path_filepath = os.path.join(base_dir, "rectangle_path.csv")
    with open(path_filepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Index", "Smoothed_UTM_X", "Smoothed_UTM_Y", "Latitude", "Longitude", "Calculated_Yaw_Deg", "Distance", "Speed"])
        for idx, pt in enumerate(path_points):
            writer.writerow([
                idx,
                f"{pt['Smoothed_UTM_X']:.3f}",
                f"{pt['Smoothed_UTM_Y']:.3f}",
                f"{pt['Latitude']:.8f}",
                f"{pt['Longitude']:.8f}",
                f"{pt['Calculated_Yaw_Deg']:.1f}",
                f"{pt['Distance']:.2f}",
                pt['Speed']
            ])
            
    print(f"[Rectangle] Generated {len(path_points)} path points (total length: {total_accumulated_dist:.2f}m).")
    print(f"[Rectangle] Dense path waypoints saved to: {path_filepath}")

if __name__ == "__main__":
    if len(sys.argv) < 9:
        print("Usage: python3 generate_rectangle.py <lat1> <lon1> <lat2> <lon2> <lat3> <lon3> <lat4> <lon4> [spacing_m] [speed_m_s]")
        print("Example (runs using local fallback directory):")
        print("  python3 generate_rectangle.py 10.791200 76.241200 10.791200 76.241380 10.791290 76.241380 10.791290 76.241200 1.0 0.15")
        sys.exit(1)
        
    lats = [float(sys.argv[i]) for i in [1, 3, 5, 7]]
    lons = [float(sys.argv[i]) for i in [2, 4, 6, 8]]
    
    space = float(sys.argv[9]) if len(sys.argv) > 9 else 1.0
    spd = float(sys.argv[10]) if len(sys.argv) > 10 else 0.15
    
    # Use current folder as base if run locally
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    generate_rectangle_path(lats[0], lons[0], lats[1], lons[1], lats[2], lons[2], lats[3], lons[3], 
                            space, spd, base_dir=current_dir)

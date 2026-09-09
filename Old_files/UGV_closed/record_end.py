#!/usr/bin/env python3
# record_end.py
# Interactive UGV Waypoint Recorder for Straight Lines, Rectangles, and Custom Polygons.

import time
import csv
import sys
import os
import math
from rtk_receiver import RTKReceiver
from localization import latlon_to_utm
from config import RTK_OFFSET_X, RTK_OFFSET_Y

def main():
    # Setup paths to import locally
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    
    rtk = RTKReceiver()
    points = []
    
    print("=====================================================================")
    print("             UGV INTERACTIVE MULTI-POINT PATH RECORDER               ")
    print("=====================================================================")
    print("Instructions:")
    print("  1. Manually drive/push the UGV to each corner or waypoint.")
    print("  2. Wait for RTK FIXED status, then press [Enter] to record.")
    print("  3. Type 'd' or 'done' and press [Enter] when you have recorded all points.")
    print("=====================================================================\n")
    
    # 1. Collection Loop
    while True:
        num = len(points) + 1
        prompt = f"Move UGV to Position #{num} and press [Enter] to record (or type 'd' when done): "
        user_input = input(prompt).strip().lower()
        
        if user_input in ['d', 'done']:
            if len(points) < 2:
                print("⚠️ Error: You must record at least 2 points to generate a path!")
                continue
            break
            
        # Poll for latest valid frame
        print("  Connecting to RTK stream... (Waiting for RTK FIXED status)", end='\r')
        timeout = time.time() + 5.0
        valid_sample = None
        
        while time.time() < timeout:
            if rtk.new_data_available:
                pose = rtk.get_latest_pose()
                if pose["fix_status"] == "RTK FIXED":
                    valid_sample = pose
                    break
            time.sleep(0.02)
            
        if valid_sample is None:
            print("\n❌ Error: Could not capture an 'RTK FIXED' point. Please try again.")
            continue
            
        # Direct conversion to UTM without inter-point EMA smoothing filter lag
        easting, northing, _ = latlon_to_utm(valid_sample["lat"], valid_sample["lon"])
        
        rtk_heading_deg = valid_sample["heading"]
        if rtk_heading_deg > 0.0:
            yaw = math.radians((90.0 - rtk_heading_deg) % 360.0)
            x = easting - RTK_OFFSET_X * math.cos(yaw) + RTK_OFFSET_Y * math.sin(yaw)
            y = northing - RTK_OFFSET_X * math.sin(yaw) - RTK_OFFSET_Y * math.cos(yaw)
        else:
            x = easting
            y = northing
        
        points.append((x, y, valid_sample["lat"], valid_sample["lon"]))
        print(f"✅ Recorded Position #{num}: UTM_X = {x:.3f}, UTM_Y = {y:.3f}, Lat = {valid_sample['lat']:.8f}, Lon = {valid_sample['lon']:.8f}")

    # Shut down socket connection
    rtk.close()

    total_corners = len(points)
    print(f"\nCaptured {total_corners} corner positions.")

    # 2. Ask to close the loop (for polygons/rectangles)
    close_loop = False
    if total_corners >= 3:
        ans = input("Do you want to close the loop back to the starting point? (y/n) [default: n]: ").strip().lower()
        close_loop = (ans == 'y' or ans == 'yes')
        
    # 3. Ask for spacing
    spacing_str = input("Enter waypoint spacing in meters [default: 1.0]: ").strip()
    try:
        spacing = float(spacing_str) if spacing_str else 1.0
    except ValueError:
        spacing = 1.0
        print("Invalid spacing. Using default: 1.0 meter.")

    # 4. Set up path segments list
    segments = list(points)
    if close_loop:
        segments.append(points[0])  # Add starting point at the end to close boundary

    # 5. Interpolate segments
    waypoints = []
    total_accumulated_dist = 0.0
    wp_idx = 0
    default_speed = 0.15  # Default target speed (m/s)

    for s in range(len(segments) - 1):
        x1, y1, lat1, lon1 = segments[s]
        x2, y2, lat2, lon2 = segments[s+1]
        
        dx = x2 - x1
        dy = y2 - y1
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-4:
            continue
            
        heading_deg = math.degrees(math.atan2(dy, dx)) % 360.0
        num_steps = max(1, int(round(seg_len / spacing)))
        
        for i in range(num_steps):
            t = i / num_steps
            xi = x1 + t * dx
            yi = y1 + t * dy
            lati = lat1 + t * (lat2 - lat1)
            loni = lon1 + t * (lon2 - lon1)
            
            waypoints.append({
                "Index": wp_idx,
                "Smoothed_UTM_X": round(xi, 3),
                "Smoothed_UTM_Y": round(yi, 3),
                "Latitude": round(lati, 8),
                "Longitude": round(loni, 8),
                "Calculated_Yaw_Deg": round(heading_deg, 1),
                "Distance": round(total_accumulated_dist + t * seg_len, 2),
                "Speed": default_speed
            })
            wp_idx += 1
            
        total_accumulated_dist += seg_len

    # Add final endpoint to the path
    x_end, y_end, lat_end, lon_end = segments[-1]
    last_yaw = waypoints[-1]["Calculated_Yaw_Deg"] if waypoints else 0.0
    
    waypoints.append({
        "Index": wp_idx,
        "Smoothed_UTM_X": round(x_end, 3),
        "Smoothed_UTM_Y": round(y_end, 3),
        "Latitude": round(lat_end, 8),
        "Longitude": round(lon_end, 8),
        "Calculated_Yaw_Deg": round(last_yaw, 1),
        "Distance": round(total_accumulated_dist, 2),
        "Speed": default_speed
    })

    # 6. Write waypoints file to target directory
    csv_path = "/home/debian/Robot/UGV_closed/waypoints_1m.csv"
    if not os.path.exists(os.path.dirname(csv_path)):
        csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "waypoints_1m.csv")

    try:
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Index", "Smoothed_UTM_X", "Smoothed_UTM_Y", "Latitude", "Longitude", "Calculated_Yaw_Deg", "Distance", "Speed"])
            for wp in waypoints:
                writer.writerow([
                    wp["Index"],
                    f"{wp['Smoothed_UTM_X']:.3f}",
                    f"{wp['Smoothed_UTM_Y']:.3f}",
                    f"{wp['Latitude']:.8f}",
                    f"{wp['Longitude']:.8f}",
                    f"{wp['Calculated_Yaw_Deg']:.1f}",
                    f"{wp['Distance']:.2f}",
                    wp['Speed']
                ])
        print(f"\n✅ Success! Generated {len(waypoints)} waypoints (total path length: {total_accumulated_dist:.2f} meters).")
        print(f"Saved file to: {csv_path}")
    except Exception as e:
        print(f"\n❌ Error writing waypoints file: {e}")

if __name__ == "__main__":
    main()
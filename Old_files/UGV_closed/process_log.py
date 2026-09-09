# process_log.py
import sys
import os
import csv
import math

def process_trajectory(log_filename):
    input_path = os.path.join("/home/debian/Robot/UGV_closed/logs", log_filename)
    output_path = "/home/debian/Robot/UGV_closed/waypoints_1m.csv"
    
    if not os.path.exists(input_path):
        print(f"Error: Log file {input_path} not found.")
        return

    waypoints = []
    last_x, last_y = None, None
    desired_spacing = 1.0  # meter
    wp_idx = 0

    with open(input_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Filter out entries where RTK did not achieve a high precision lock
            if row["RTK_Fix_Status"] != "RTK FIXED":
                continue
                
            try:
                # Target the online smoothed variables produced by your logger
                x = float(row["Smoothed_UTM_X"])
                y = float(row["Smoothed_UTM_Y"])
                lat = float(row.get("Smoothed_Latitude", row.get("Latitude", 0.0)))
                lon = float(row.get("Smoothed_Longitude", row.get("Longitude", 0.0)))
            except ValueError:
                continue

            if last_x is None:
                waypoints.append((wp_idx, x, y, lat, lon))
                wp_idx += 1
                last_x, last_y = x, y
            else:
                # Calculate physical displacement distance
                dist = math.hypot(x - last_x, y - last_y)
                if dist >= desired_spacing:
                    waypoints.append((wp_idx, x, y, lat, lon))
                    wp_idx += 1
                    last_x, last_y = x, y

    # Write out the clean structural waypoints list
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Index", "Smoothed_UTM_X", "Smoothed_UTM_Y", "Latitude", "Longitude"])
        for wp in waypoints:
            writer.writerow([wp[0], f"{wp[1]:.3f}", f"{wp[2]:.3f}", f"{wp[3]:.8f}", f"{wp[4]:.8f}"])

    print(f"Successfully processed {log_filename} -> Extracted {len(waypoints)} straight line path points.")
    print(f"Saved path configuration to: {output_path}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 process_log.py <name_of_csv_log_file.csv>")
    else:
        process_trajectory(sys.argv[1])
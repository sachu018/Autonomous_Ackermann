#!/usr/bin/env python3
# make_circle_path.py — interactively generate a circle path CSV.
# Usage: python3 make_circle_path.py

import os

import rover_config as cfg
import waypoints as wp
from path_prompts import prompt_float, announce_saved

if __name__ == "__main__":
    print("=== Circle path generator ===")
    diameter_m = prompt_float("Diameter (m): ", min_val=0.0)

    if diameter_m / 2.0 < cfg.MIN_TURN_RADIUS_M:
        print(f"NOTE: radius {diameter_m/2.0:.2f}m is tighter than this "
              f"chassis's minimum turn radius ({cfg.MIN_TURN_RADIUS_M:.2f}m) "
              f"— this circle will not actually be drivable. Continuing "
              f"anyway; waypoints.py will also warn when the path is used.")

    path = wp.generate_circle(diameter_m)

    os.makedirs("paths", exist_ok=True)
    out_path = f"paths/circle_d{diameter_m:g}m.csv"
    wp.save_csv(path, out_path)
    announce_saved(out_path, len(path))

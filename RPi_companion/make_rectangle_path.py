#!/usr/bin/env python3
# make_rectangle_path.py — interactively generate a rectangle path CSV.
# Usage: python3 make_rectangle_path.py

import os

import waypoints as wp
from path_prompts import prompt_float, announce_saved

if __name__ == "__main__":
    print("=== Rectangle path generator ===")
    width_m = prompt_float("Length / width, along the starting heading (m): ", min_val=0.0)
    height_m = prompt_float("Breadth / height, perpendicular (m): ", min_val=0.0)

    path = wp.generate_rectangle(width_m, height_m)

    os.makedirs("paths", exist_ok=True)
    out_path = f"paths/rectangle_{width_m:g}x{height_m:g}m.csv"
    wp.save_csv(path, out_path)
    announce_saved(out_path, len(path))

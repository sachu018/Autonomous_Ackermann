#!/usr/bin/env python3
# make_straight_path.py — interactively generate a straight-line path CSV.
# Usage: python3 make_straight_path.py

import os

import waypoints as wp
from path_prompts import prompt_float, announce_saved

if __name__ == "__main__":
    print("=== Straight line path generator ===")
    length_m = prompt_float("Length (m): ", min_val=0.0)

    path = wp.generate_straight_line(length_m)

    os.makedirs("paths", exist_ok=True)
    out_path = f"paths/straight_{length_m:g}m.csv"
    wp.save_csv(path, out_path)
    announce_saved(out_path, len(path))

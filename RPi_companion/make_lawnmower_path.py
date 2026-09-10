#!/usr/bin/env python3
# make_lawnmower_path.py — interactively generate a lawnmower path CSV.
# Usage: python3 make_lawnmower_path.py

import os

import rover_config as cfg
import waypoints as wp
from path_prompts import prompt_float, prompt_int, announce_saved

if __name__ == "__main__":
    print("=== Lawnmower path generator ===")
    row_length_m = prompt_float("Row length (m): ", min_val=0.0)
    num_rows = prompt_int("Number of rows: ", min_val=2)

    path = wp.generate_lawnmower(row_length_m, num_rows)

    os.makedirs("paths", exist_ok=True)
    out_path = f"paths/lawnmower_{row_length_m:g}m_x{num_rows}rows.csv"
    wp.save_csv(path, out_path)
    announce_saved(out_path, len(path))

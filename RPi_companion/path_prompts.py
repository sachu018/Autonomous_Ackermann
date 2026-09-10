#!/usr/bin/env python3
# path_prompts.py — tiny shared input helper for the make_*_path.py scripts.
# Not a program on its own.

import sys


def prompt_float(message, min_val=None):
    while True:
        raw = input(message).strip()
        try:
            val = float(raw)
        except ValueError:
            print("  Not a number, try again.")
            continue
        if min_val is not None and val <= min_val:
            print(f"  Must be greater than {min_val}, try again.")
            continue
        return val


def prompt_int(message, min_val=None):
    while True:
        raw = input(message).strip()
        try:
            val = int(raw)
        except ValueError:
            print("  Not a whole number, try again.")
            continue
        if min_val is not None and val < min_val:
            print(f"  Must be at least {min_val}, try again.")
            continue
        return val


def announce_saved(path, n_waypoints):
    print(f"\nSaved {n_waypoints} waypoints to: {path}")
    print(f"Use it with: python3 mission.py --csv {path}\n")


if __name__ == "__main__":
    print("This is a helper module, not a program — run one of the "
          "make_*_path.py scripts instead (make_straight_path.py, "
          "make_rectangle_path.py, make_circle_path.py, "
          "make_lawnmower_path.py).")
    sys.exit(1)

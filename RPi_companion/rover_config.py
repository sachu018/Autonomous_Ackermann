#!/usr/bin/env python3
# rover_config.py — shared tunables for the RPi-side autonomous stack (Phase 5).
#
# Mirrors the relevant constants from the STM32 firmware
# (Rover_closed_loop/Core/Inc/ackermann_config.h, config.h) — there is no
# shared header between the two languages/processors, so these are kept in
# sync BY HAND. If the STM32 side's WHEELBASE/MAX_STEER/WHEEL_RADIUS ever
# change, update them here too. Same caveat as uart_link.py's frame formats.
#
# Parameter provenance — see scratchpad.md and the architecture.md
# discussion for the reasoning behind each value, including what carried
# over from Old_files/UGV_closed/config.py's field-tuned numbers and what
# didn't (that system had RTK position noise + solid dead-reckoning-free
# heading; ours is the opposite: solid IMU heading, accumulating dead-
# reckoning position drift with no absolute correction).

import math

# ── Chassis geometry (mirror ackermann_config.h / config.h) ────────────────
WHEELBASE_M = 0.80           # ACK_WHEELBASE_M — front axle to rear axle
MAX_STEER_ANGLE_DEG = 45.0   # MAX_STEER_ANGLE_DEG — physical steering lock
WHEEL_RADIUS_M = 0.175       # WHEEL_RADIUS_M — for encoder RPM -> distance

# Minimum radius this chassis can actually turn at full steering lock.
# R = L / tan(delta_max). Used by waypoints.py to catch/flag geometrically
# undrivable turns (e.g. a lawnmower U-turn tighter than this) instead of
# silently generating a path the rover cannot follow.
MIN_TURN_RADIUS_M = WHEELBASE_M / math.tan(math.radians(MAX_STEER_ANGLE_DEG))

# ── Path generation (waypoints.py) ──────────────────────────────────────────
SPACING_STRAIGHT_M = 1.0        # waypoint spacing on straight segments
SPACING_CURVE_M = 0.3           # waypoint spacing on curves (circle, turns)
LAWNMOWER_ROW_SPACING_M = 1.0   # nominal distance between lawnmower rows

# ── Guidance / Pure Pursuit (guidance.py) ───────────────────────────────────
LOOKAHEAD_M = 0.8           # Ld — same value as the old system's LOOK_AHEAD_DIST
WP_REACHED_THRESH_M = 0.25  # "arrived" / mission-end distance threshold
YAW_BOUND_DEG = 25.0        # clamp on heading error before it drives speed shaping
SPEED_COS_FLOOR = 0.2       # speed *= max(this, cos(bounded heading error))

# ── Speed profile ────────────────────────────────────────────────────────────
# Placeholders, not yet field-tuned — see scratchpad.md. Retune from logged
# cross-track error once this runs on hardware, same way the old system's
# comments show its own gains were arrived at ("tuned for < 2.5cm mean CTE").
CRUISE_SPEED_MPS = 0.2
MIN_SPEED_MPS = 0.05

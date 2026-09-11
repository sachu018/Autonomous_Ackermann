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
YAW_BOUND_DEG = 25.0        # clamp on heading error before it drives the STEERING law

# ── Speed profile ────────────────────────────────────────────────────────────
# First real field run (mission_20260911_113236/114042) surfaced two things:
# (1) speed barely varied (stayed ~0.18-0.20 m/s) even while badly off track
#     (alpha hit 157 deg at one point) — because the speed law reused
#     YAW_BOUND_DEG (25 deg, meant for the STEERING law) as its own error
#     input: cos(25 deg)=0.906, so speed could never drop more than ~9% no
#     matter how large the REAL error got. Fixed by giving speed shaping its
#     own, much wider error thresholds below, decoupled from YAW_BOUND_DEG.
# (2) 0.2 m/s cruise felt too fast in the field. Lowered — still a starting
#     point pending further field tuning, not a validated final value.
CRUISE_SPEED_MPS = 0.12

# Below this |speed_target_ms|, the STM32 brakes instead of creeping
# (AUTO_SPEED_DEADBAND_MS in ackermann_config.h = 0.03 m/s) — so this floor
# only needs to clear that with some margin. The STM32 also has its own
# stiction-compensated throttle floor (THR_FWD_MIN_DAC) that kicks in for
# ANY nonzero commanded speed, so a genuine stall at this floor is unlikely
# — but this hasn't been directly exercised/confirmed on real hardware yet
# (this run's speed never dropped anywhere near this low) — treat as
# provisional until a field test actually drives at MIN_SPEED_MPS for a
# sustained stretch and confirms the wheels keep turning.
MIN_SPEED_MPS = 0.06

# Speed shaping: reduce speed as EITHER heading error (to the lookahead
# point) or cross-track error grows — responsive to how far off-track the
# rover actually is, not just a near-constant cruise regardless of error.
# Each error ramps speed linearly down to MIN_SPEED_FACTOR (a fraction of
# target speed, not a hard stop — see MIN_SPEED_MPS above for why never
# fully stopping matters) as it approaches its own "FULL" threshold; the
# worse of the two errors wins (min of the two factors), not a product of
# both (a product would double-penalize a rover that's simultaneously a
# bit off-heading AND a bit off-track, dropping speed further than either
# error alone would justify).
SPEED_ALPHA_FULL_DEG = 60.0     # |alpha| at/beyond which speed bottoms out
SPEED_CTE_FULL_M = LOOKAHEAD_M  # |cte| at/beyond which speed bottoms out — being
                                 # off by about one lookahead distance means the
                                 # rover is essentially lost, not just imprecise
MIN_SPEED_FACTOR = 0.3          # floor fraction of target speed even at max error

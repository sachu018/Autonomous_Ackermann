#!/usr/bin/env python3
# calibrate_steering_pots.py — re-measure ADC_L/R_MIN/CENTER/MAX_RAW for
# ackermann_config.h, with BOTH wheels checked against one shared physical
# reference at the center position (the previous recalibration measured
# each wheel's "straight" independently, which left a small but consistent
# ~1-1.6 deg L/R disagreement at the center point — see scratchpad.md).
#
# This script does NOT read raw ADC counts directly (those aren't in the
# UART feedback frame — see rpi_link.c's frame format). Instead it reads
# the STM32's already-computed angle_L_deg/angle_R_deg (from the CURRENT,
# imperfect calibration in the firmware right now) and inverts _MapToAngle()
# using those same current constants to recover the implied raw ADC value
# at whatever physical position the wheels are actually in. This is exact,
# not an approximation — inverting a piecewise-linear map with its own
# constants recovers the true input exactly, up to the angle's transmitted
# quantization (0.01 deg over a ~45 deg span across ~1000-1400 raw counts —
# far finer than 1 raw count, negligible).
#
# ⚠️ CURRENT_* below MUST match Rover_closed_loop/Core/Inc/ackermann_config.h
# at the time this is run, or the inversion will be wrong. Update them by
# hand if the firmware's values changed since this was written.
#
# Usage:
#   source ~/rover-venv/bin/activate
#   python3 calibrate_steering_pots.py
#
# Walks through: full LEFT lock, full RIGHT lock, then CENTER (checked with
# a string-line/straightedge across BOTH front wheels at once, not judged
# per wheel) — printing live angle_L/angle_R/delta continuously so you can
# watch the wheels agree before capturing each point. Ends by printing the
# 6 new ADC_L/R_MIN/CENTER/MAX_RAW values ready to paste into
# ackermann_config.h.

import select
import sys
import time

from uart_link import STM32Link

# ── Must match ackermann_config.h at the moment this is run ────────────────
CURRENT_MAX_STEER_ANGLE_DEG = 45.0
CURRENT_L = dict(min_raw=3096, center_raw=4426, max_raw=5793)
CURRENT_R = dict(min_raw=2124, center_raw=3167, max_raw=4160)


def angle_to_raw(angle_deg, cal):
    """Exact inverse of ads1115.c's _MapToAngle(), using the CURRENT
    (possibly imperfect) calibration constants that produced this angle."""
    if angle_deg >= 0.0:
        norm = angle_deg / CURRENT_MAX_STEER_ANGLE_DEG
        return cal["center_raw"] + norm * (cal["max_raw"] - cal["center_raw"])
    else:
        norm = -angle_deg / CURRENT_MAX_STEER_ANGLE_DEG
        return cal["center_raw"] - norm * (cal["center_raw"] - cal["min_raw"])


def _enter_pressed():
    """Non-blocking check for a pending Enter keypress on stdin."""
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if ready:
        sys.stdin.readline()
        return True
    return False


def wait_for_capture(link, label, hint):
    print(f"\n--- {label} ---")
    print(hint)
    print("Watching live angle_L / angle_R / delta. Press Enter when in position.\n")
    while True:
        d = link.read()
        delta = (d.angle_L_deg + d.angle_R_deg) / 2.0
        print(
            f"\r  angle_L={d.angle_L_deg:>+7.2f}  angle_R={d.angle_R_deg:>+7.2f}  "
            f"delta={delta:>+7.2f}  (L-R={d.angle_L_deg - d.angle_R_deg:>+6.2f})   ",
            end="", flush=True,
        )
        if _enter_pressed():
            print()  # move off the \r line
            return d.angle_L_deg, d.angle_R_deg
        time.sleep(0.05)


def main():
    link = STM32Link()
    if not link.is_open():
        print("[Calibrate] Could not open the serial port — see the error above.")
        sys.exit(1)

    print(
        "\nSteering pot recalibration.\n"
        "Place the rover on flat, level ground with the drive UNPOWERED or "
        "the area clear — you'll be moving the wheels by hand/actuator.\n"
        "This reuses the CURRENT firmware calibration's angle readout and "
        "inverts it to recover raw ADC counts, so it works over the UART "
        "link with no extra firmware changes.\n"
    )

    aL_left, aR_left = wait_for_capture(
        link, "1/3 — FULL LEFT LOCK",
        "Turn the wheels (via the actuator/manual stick) to full left lock.",
    )
    aL_right, aR_right = wait_for_capture(
        link, "2/3 — FULL RIGHT LOCK",
        "Turn the wheels to full right lock.",
    )
    aL_center, aR_center = wait_for_capture(
        link, "3/3 — CENTER (dead straight)",
        "Run a string-line or straightedge across BOTH front tires at once "
        "and confirm BOTH are simultaneously flush against it — not judged "
        "wheel-by-wheel. This is the step the previous calibration round "
        "got slightly wrong (see scratchpad.md).",
    )

    link.close()

    # ADC_*_MIN_RAW = full RIGHT lock, ADC_*_MAX_RAW = full LEFT lock (the
    # sign convention documented in ackermann_config.h) — named explicitly
    # by which lock they came from, not by numeric min/max, to avoid mixing
    # them up if this script is ever edited.
    raw_L_right_lock = angle_to_raw(aL_right, CURRENT_L)
    raw_L_left_lock = angle_to_raw(aL_left, CURRENT_L)
    raw_L_center = angle_to_raw(aL_center, CURRENT_L)
    raw_R_right_lock = angle_to_raw(aR_right, CURRENT_R)
    raw_R_left_lock = angle_to_raw(aR_left, CURRENT_R)
    raw_R_center = angle_to_raw(aR_center, CURRENT_R)

    print("\n" + "=" * 70)
    print("New calibration values — paste into ackermann_config.h:")
    print("=" * 70)
    print(f"#define ADC_L_MIN_RAW           {round(raw_L_right_lock)}    "
          f"/* Left  pot @ full RIGHT lock */")
    print(f"#define ADC_L_MAX_RAW           {round(raw_L_left_lock)}    "
          f"/* Left  pot @ full LEFT  lock */")
    print(f"#define ADC_R_MIN_RAW           {round(raw_R_right_lock)}    "
          f"/* Right pot @ full RIGHT lock */")
    print(f"#define ADC_R_MAX_RAW           {round(raw_R_left_lock)}    "
          f"/* Right pot @ full LEFT  lock */")
    print(f"#define ADC_L_CENTER_RAW        {round(raw_L_center)}")
    print(f"#define ADC_R_CENTER_RAW        {round(raw_R_center)}")
    print("=" * 70)

    l_span_right = raw_L_center - raw_L_right_lock  # center -> right lock
    l_span_left = raw_L_left_lock - raw_L_center     # center -> left lock
    r_span_right = raw_R_center - raw_R_right_lock
    r_span_left = raw_R_left_lock - raw_R_center
    print(f"\nLeft  span: right={abs(l_span_right):.0f}  left={abs(l_span_left):.0f}")
    print(f"Right span: right={abs(r_span_right):.0f}  left={abs(r_span_left):.0f}")
    print(f"Center-point L/R angle gap, under the OLD (current) calibration, "
          f"at the position you just marked as true center: "
          f"{aL_center - aR_center:+.2f} deg")
    print(
        "\nThat gap is exactly the kind of ~1.1-1.6 deg mismatch this "
        "recalibration is meant to fix — it will NOT read ~0 until the new "
        "values above are pasted into ackermann_config.h and reflashed. "
        "After reflashing, re-run this script (or read_encoders.py) at the "
        "same true-straight position and confirm angle_L/angle_R now agree "
        "closely — if they still don't, the string-line check at step 3 "
        "may not have been simultaneous for both wheels; repeat it.\n"
    )


if __name__ == "__main__":
    main()

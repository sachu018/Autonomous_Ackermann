#!/usr/bin/env python3
# calibrate_imu.py — BNO055 calibration monitor + offset dump.
#
# Run this once after mounting the sensor on the rover (and again any time
# it's remounted or moved — calibration offsets are specific to this exact
# physical placement and the local magnetic environment, they do NOT
# transfer from the old BBB rig's saved values).
#
# Unlike Old_files/dev_bak/calibrate_imu.py (which only watched Gyro+Accel
# and gave up after 15s), this version waits for full calibration (Sys,
# Gyro, Accel, Mag all = 3) and then reads back and prints the sensor's
# internal offset registers — paste that block into imu.py's
# IMU_CALIBRATION_OFFSETS if/when we add offset-preload support there.
#
# Usage:
#   source ~/rover-venv/bin/activate
#   python3 calibrate_imu.py

import sys
import time

import board
import adafruit_bno055


def main():
    print("[Calibration] Connecting to BNO055...")
    try:
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c)
    except Exception as e:
        print(f"[Calibration] Failed to connect: {e}")
        sys.exit(1)

    print("[Calibration] Move the sensor as follows until all four reach 3:")
    print("  - Gyroscope:      leave the sensor perfectly still for a few seconds")
    print("  - Accelerometer:  slowly place it in ~6 distinct orientations "
          "(each axis up/down), pausing ~2s each")
    print("  - Magnetometer:   move it in a slow figure-8, away from motors/metal")
    print("  - System:         usually locks in shortly after the other three\n")

    start = time.time()
    last_cal = (-1, -1, -1, -1)
    try:
        while True:
            cal = sensor.calibration_status  # (sys, gyro, accel, mag)
            if cal != last_cal:
                print(f"  Sys={cal[0]} Gyro={cal[1]} Accel={cal[2]} Mag={cal[3]}"
                      f"   (t={time.time()-start:4.0f}s)")
                last_cal = cal

            if cal[0] == 3 and cal[1] == 3 and cal[2] == 3 and cal[3] == 3:
                print("\n[Calibration] FULLY CALIBRATED.")
                break

            if time.time() - start > 180:
                print("\n[Calibration] Timeout (180s) — not fully calibrated. "
                      "Re-run and try more deliberate motion, especially the "
                      "figure-8 for the magnetometer.")
                sys.exit(1)

            time.sleep(0.3)
    except KeyboardInterrupt:
        print("\n[Calibration] Interrupted by user — not saving offsets.")
        sys.exit(1)

    accel_off = sensor.offsets_accelerometer
    mag_off = sensor.offsets_magnetometer
    gyro_off = sensor.offsets_gyroscope
    accel_rad = sensor.radius_accelerometer
    mag_rad = sensor.radius_magnetometer

    print("\n[Calibration] Offsets (specific to THIS sensor + THIS mounting).")
    print("Paste this into imu.py's IMU_CALIBRATION_OFFSETS:\n")
    print("IMU_CALIBRATION_OFFSETS = {")
    print(f"    \"accel_offset\": {tuple(accel_off)},")
    print(f"    \"mag_offset\": {tuple(mag_off)},")
    print(f"    \"gyro_offset\": {tuple(gyro_off)},")
    print(f"    \"accel_radius\": {accel_rad},")
    print(f"    \"mag_radius\": {mag_rad},")
    print("}")
    print("\n[Calibration] Also record these in scratchpad.md so they're not "
          "lost if the sensor is ever remounted and re-calibrated.")


if __name__ == "__main__":
    main()

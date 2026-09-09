#!/usr/bin/env python3
# test_imu.py — live BNO055 readout. Run this FIRST after wiring the sensor,
# before trusting imu.py in anything else, to confirm the hardware/wiring/
# I2C address are all correct on this specific Raspberry Pi 5.
#
# Adapted from Old_files/dev_bak/test_imu.py (BBB version) for the RPi5 +
# imu.py's IMUData wrapper — see architecture.md §3 for context.
#
# Usage:
#   source ~/rover-venv/bin/activate
#   python3 test_imu.py

import time
import sys

from imu import IMU


def main():
    imu = IMU()
    if not imu.is_healthy():
        print("[Test] IMU did not initialize — check wiring (3V3/SDA/SCL/GND) "
              "and run: i2cdetect -y 1   (expect a device at 0x28 or 0x29)")
        sys.exit(1)

    print("\nReading IMU data. Press Ctrl+C to stop.\n")
    print(f"{'Heading':>8} {'Roll':>7} {'Pitch':>7} | {'GyroZ':>8} | "
          f"{'AccelX':>7} {'AccelY':>7} | {'Cal S/G/A/M':>14} | Valid")
    print("-" * 80)

    try:
        while True:
            d = imu.read()
            print(
                f"{d.heading_deg:>8.1f} {d.roll_deg:>7.1f} {d.pitch_deg:>7.1f} | "
                f"{d.gyro_z_rads:>8.3f} | "
                f"{d.accel_x_ms2:>7.2f} {d.accel_y_ms2:>7.2f} | "
                f"{d.calib_sys} {d.calib_gyro} {d.calib_accel} {d.calib_mag}"
                f"{'':>9} | {'OK' if d.valid else 'FAULT'}",
                end="\r", flush=True,
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\n[Test] Stopped by user.")


if __name__ == "__main__":
    main()

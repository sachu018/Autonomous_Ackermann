#!/usr/bin/env python3
"""
run_magnetometer_calibration.py
------------------------------
Runs the BNO055 IMU in 9-axis NDOF mode, displays live calibration status,
and saves the offsets to /home/debian/Robot/bno055_calibration.json when exited.
"""
import time
import board
import adafruit_bno055
import json
import os

def main():
    print("Initializing BNO055 I2C interface...")
    try:
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c)
        sensor.mode = 0x0C  # Set to 9-axis NDOF mode
    except Exception as e:
        print(f"Error: Failed to initialize BNO055: {e}")
        return

    print("\n--- 9-Axis Magnetometer Calibration ---")
    print("1. Rotate the UGV slowly in a figure-8 pattern to calibrate the magnetometer.")
    print("2. Tilt the UGV slowly to calibrate the accelerometer.")
    print("3. Keep the UGV stationary for 2-3 seconds to calibrate the gyroscope.")
    print("4. Press Ctrl+C once Magnetometer (Mag) calibration reaches 3 to save offsets.")
    print("-" * 50)

    try:
        while True:
            sys_c, gyro_c, accel_c, mag_c = sensor.calibration_status
            print(f"Calibration Status -> System: {sys_c}/3 | Gyro: {gyro_c}/3 | Accel: {accel_c}/3 | Mag: {mag_c}/3", end='\r')
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n\nReading calibration offsets from sensor registers...")
        try:
            offsets = {
                "accel": list(sensor.offsets_accelerometer),
                "gyro": list(sensor.offsets_gyroscope),
                "mag": list(sensor.offsets_magnetometer),
                "accel_radius": sensor.radius_accelerometer,
                "mag_radius": sensor.radius_magnetometer
            }
            
            cal_file = "/home/debian/Robot/bno055_calibration.json"
            with open(cal_file, "w") as f:
                json.dump(offsets, f, indent=4)
            print(f"✓ Calibration successfully saved to {cal_file}")
            print(offsets)
        except Exception as e:
            print(f"Failed to read/save offsets: {e}")

if __name__ == "__main__":
    main()

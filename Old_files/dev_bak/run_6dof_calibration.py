#!/usr/bin/env python3
"""
run_6dof_calibration.py
-----------------------
Runs the BNO055 IMU in 6-DOF IMU mode, displays live Gyro & Accel calibration status,
and saves the offsets to /home/debian/Robot/bno055_calibration.json.
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
        sensor.mode = 0x08  # Set to 6-DOF IMU mode
    except Exception as e:
        print(f"Error: Failed to initialize BNO055: {e}")
        return

    print("\n--- 6-DOF Calibration Script ---")
    print("1. Keep the UGV COMPLETELY STILL on a flat surface (crucial for gyroscope calibration).")
    print("2. Tilt the UGV slightly if you want to calibrate the accelerometer.")
    print("3. Once Gyroscope calibration reaches 3/3, press Ctrl+C to save offsets.")
    print("-" * 60)

    try:
        while True:
            sys_c, gyro_c, accel_c, mag_c = sensor.calibration_status
            print(f"Calibration Status -> System: {sys_c}/3 | Gyro: {gyro_c}/3 | Accel: {accel_c}/3 | (Mag Ignored in 6-DOF)", end='\r')
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n\nReading calibration offsets from sensor registers...")
        try:
            offsets = {
                "accel": list(sensor.offsets_accelerometer),
                "gyro": list(sensor.offsets_gyroscope),
                "mag": [0, 0, 0],             # Saved as 0 in 6-DOF mode
                "accel_radius": sensor.radius_accelerometer,
                "mag_radius": 0               # Saved as 0 in 6-DOF mode
            }
            
            cal_file = "/home/debian/Robot/bno055_calibration.json"
            with open(cal_file, "w") as f:
                json.dump(offsets, f, indent=4)
            print(f"✓ 6-DOF calibration successfully saved to {cal_file}")
            print(offsets)
        except Exception as e:
            print(f"Failed to read/save offsets: {e}")

if __name__ == "__main__":
    main()

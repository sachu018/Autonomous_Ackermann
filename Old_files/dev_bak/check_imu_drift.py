import time
import board
import adafruit_bno055
import os
import json
import math

def load_calibration(sensor):
    cal_file = "/home/debian/Robot/bno055_calibration.json"
    if not os.path.exists(cal_file):
        print("[Drift Test] No calibration file found.")
        return
    try:
        with open(cal_file, "r") as f:
            offsets = json.load(f)
        sensor.offsets_accelerometer = tuple(offsets["accel"])
        sensor.offsets_gyroscope     = tuple(offsets["gyro"])
        sensor.offsets_magnetometer  = tuple(offsets["mag"])
        if "accel_radius" in offsets: sensor.radius_accelerometer = offsets["accel_radius"]
        if "mag_radius" in offsets: sensor.radius_magnetometer  = offsets["mag_radius"]
        print(f"[Drift Test] Calibration offsets successfully restored from {cal_file}")
    except Exception as e:
        print(f"[Drift Test] Warning: Could not restore calibration: {e}")

def main():
    print("[Drift Test] Initializing BNO055 I2C interface...")
    try:
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c)
        sensor.mode = 0x08  # 6-DOF IMU mode (magnetic immune)
        load_calibration(sensor)
    except Exception as e:
        print(f"[Drift Test] Failed to initialize BNO055: {e}")
        return

    # Let the sensor settle
    time.sleep(1.0)
    
    print("\n--- Starting 15-Second Stationary Drift Test ---")
    print("Keep UGV completely still...")
    
    yaw_readings = []
    cal_statuses = []
    
    start_time = time.time()
    duration = 15.0  # seconds
    
    while (time.time() - start_time) < duration:
        euler = sensor.euler
        cal = sensor.calibration_status  # Returns (sys, gyro, accel, mag)
        
        if euler is not None and euler[0] is not None:
            yaw_readings.append(euler[0])
            cal_statuses.append(cal)
            
        time.sleep(0.1)  # 10 Hz
        
    if not yaw_readings:
        print("[Drift Test] Error: No yaw readings could be gathered.")
        return
        
    start_yaw = yaw_readings[0]
    end_yaw = yaw_readings[-1]
    
    # Calculate difference, handling 360-degree wrap around
    diff_yaw = end_yaw - start_yaw
    if diff_yaw > 180.0: diff_yaw -= 360.0
    elif diff_yaw < -180.0: diff_yaw += 360.0
    
    drift_rate_per_min = (diff_yaw / duration) * 60.0
    
    # Get final calibration levels
    sys_cal, gyro_cal, accel_cal, mag_cal = cal_statuses[-1]
    
    print("\n--- Drift Test Results ---")
    print(f"Test Duration: {duration} seconds")
    print(f"Start Yaw: {start_yaw:.2f}°")
    print(f"End Yaw:   {end_yaw:.2f}°")
    print(f"Total Yaw Drift: {diff_yaw:+.4f}°")
    print(f"Estimated Drift Rate: {drift_rate_per_min:+.4f}°/minute")
    print(f"\nFinal BNO055 Calibration Status (0-3 scale):")
    print(f"  - System: {sys_cal}/3")
    # In 6-DOF IMU mode, Mag cal status is expected to be 0 since it is disabled
    print(f"  - Gyroscope: {gyro_cal}/3")
    print(f"  - Accelerometer: {accel_cal}/3")
    print(f"  - Magnetometer: {mag_cal}/3 (Disabled in 6-DOF mode)")

if __name__ == "__main__":
    main()

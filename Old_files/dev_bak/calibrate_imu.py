#!/usr/bin/env python3
import time
import sys
import board
import adafruit_bno055

def main():
    print("[Calibration] Connecting to BNO055 IMU...")
    try:
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c)
    except Exception as e:
        print(f"[Calibration] Failed to connect: {e}")
        sys.exit(1)

    print("[Calibration] Monitoring BNO055 calibration status. Please move the sensor:")
    print("  - Gyroscope: Keep the sensor still for a few seconds.")
    print("  - Magnetometer: Move the sensor in a figure-8 motion.")
    print("  - Accelerometer: Place the sensor in different 90-degree orientations.")
    print("\nCalibration status (System, Gyro, Accel, Mag):")
    
    start_time = time.time()
    try:
        while True:
            cal = sensor.calibration_status
            print(f"  Status: Sys={cal[0]} Gyro={cal[1]} Accel={cal[2]} Mag={cal[3]}", end='\r', flush=True)
            
            if cal[1] == 3 and cal[2] == 3:
                print("\n[Calibration] Gyro and Accelerometer are fully calibrated!")
                break
                
            if time.time() - start_time > 15:
                print("\n[Calibration] Timeout reached. Moving on with current calibration.")
                break
                
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[Calibration] Interrupted by user.")
    
    print("[Calibration] BNO055 calibration monitoring completed successfully.")

if __name__ == '__main__':
    main()

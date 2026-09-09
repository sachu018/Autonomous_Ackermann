#!/usr/bin/env python3
import time
import sys
import board
import adafruit_bno055

def main():
    print("[IMU] Connecting to BNO055 IMU via board.I2C()...")
    try:
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c)
        print("[IMU] BNO055 initialized and active.")
    except Exception as e:
        print(f"[IMU] Failed to initialize BNO055: {e}")
        sys.exit(1)

    print("\nReading IMU data. Press Ctrl+C to stop.\n")
    print(f"{'Accel (g)':^30} | {'Gyro (deg/s)':^30} | {'Mag (uT)':^30} | {'Calibration':^15}")
    print(f"{'X':^9} {'Y':^9} {'Z':^9} | {'X':^9} {'Y':^9} {'Z':^9} | {'X':^9} {'Y':^9} {'Z':^9} | {'S G A M':^15}")
    print("-" * 115)

    try:
        while True:
            accel = sensor.acceleration
            gyro = sensor.gyro
            mag = sensor.magnetic
            cal = sensor.calibration_status
            
            if accel is not None and gyro is not None and mag is not None and cal is not None:
                if accel[0] is not None and gyro[0] is not None and mag[0] is not None:
                    ax, ay, az = [val / 9.80665 for val in accel]
                    gx, gy, gz = [val * 57.29577951308232 for val in gyro]
                    mx, my, mz = mag
                    cal_str = f"{cal[0]} {cal[1]} {cal[2]} {cal[3]}"
                    print(f"{ax:>+9.3f} {ay:>+9.3f} {az:>+9.3f} | {gx:>+9.2f} {gy:>+9.2f} {gz:>+9.2f} | {mx:>+9.1f} {my:>+9.1f} {mz:>+9.1f} | {cal_str:^15}", end='\r', flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\n[IMU] Stopped by user.")

if __name__ == '__main__':
    main()

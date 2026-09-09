#!/usr/bin/env python3
"""
BBB_waypoint_follower.py
------------------------
BeagleBone Black closed-loop waypoint follower.
- Reads CSV path.
- ALIGNMENT PHASE: Rotates in place to face WP 1.
- TRACKING PHASE: Drives through remaining waypoints.
- LOGGING: Saves all hardware states to a timestamped CSV.
"""

import time
import math
import sys
import os
import signal
import csv
import json
import socket
from datetime import datetime
import numpy as np

# Hardware libraries
try:
    from smbus2 import SMBus
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False

try:
    import Adafruit_BBIO.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

# ==========================================
# ROBOT CONFIGURATION & KINEMATICS
# ==========================================
a = 0.175             # Wheel radius (m)
d = 0.30              # Half wheelbase (m) - L/2
CPR = 2400            # Encoder counts per revolution
GEAR_RATIO = 20.0     # Gear ratio (1:20)
dt = 0.05             # Control loop rate (20 Hz)

WHEEL_RPM_PHYS_MAX = 10.0 
DAC_MAX_VALUE = 4095
DAC_ZERO_VALUE = 0

DAC_LEFT_ADDR = 0x61
DAC_RIGHT_ADDR = 0x60

# GPIO Pins
GPIO_CONTACTOR = "P8_14"
GPIO_REV_LEFT = "P8_7"
GPIO_REV_RIGHT = "P8_9"
GPIO_BRAKE_LEFT = "P8_8"
GPIO_BRAKE_RIGHT = "P8_10"

LEFT_MOTOR_DIR = 1
RIGHT_MOTOR_DIR = -1

# ==========================================
# WAYPOINT CONTROLLER PARAMETERS
# ==========================================
Kp_lin = 1.3
Kp_ang = 4.0
max_lin = 0.25      # Max linear speed (m/s)
max_ang = 1.5       # Max angular speed (rad/s)
wp_thresh = 0.03    # Distance threshold to consider WP reached (m)
align_thresh = 0.05 # Alignment threshold before driving (~2.8 degrees)

# ==========================================
# HARDWARE CLASSES 
# ==========================================
class BBBHardware:
    def __init__(self):
        self.bus = SMBus(2) if HW_AVAILABLE else None
        self.is_energized = False
        self.last_dac_L = 0
        self.last_dac_R = 0
        self.closed = False
        
        if GPIO_AVAILABLE:
            for pin in [GPIO_REV_LEFT, GPIO_REV_RIGHT, GPIO_BRAKE_LEFT, GPIO_BRAKE_RIGHT, GPIO_CONTACTOR]:
                os.system(f"config-pin {pin} gpio >/dev/null 2>&1 || true")
                GPIO.setup(pin, GPIO.OUT)
            
            GPIO.output(GPIO_CONTACTOR, GPIO.LOW)
            GPIO.output(GPIO_REV_LEFT, GPIO.LOW)
            GPIO.output(GPIO_REV_RIGHT, GPIO.LOW)
            GPIO.output(GPIO_BRAKE_LEFT, GPIO.HIGH)
            GPIO.output(GPIO_BRAKE_RIGHT, GPIO.HIGH)
            self.energize()
            
    def energize(self):
        if self.is_energized: return
        if GPIO_AVAILABLE:
            GPIO.output(GPIO_CONTACTOR, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(GPIO_BRAKE_LEFT, GPIO.LOW)
            GPIO.output(GPIO_BRAKE_RIGHT, GPIO.LOW)
            time.sleep(0.1)
        self.is_energized = True

    def write_dac(self, addr, value):
        if not self.bus: return
        value = max(0, min(value, DAC_MAX_VALUE))
        upper = (value >> 4) & 0xFF
        lower = (value << 4) & 0xFF
        try:
            self.bus.write_i2c_block_data(addr, 0x40, [upper, lower])
        except Exception:
            pass

    def drive_motors(self, rpm_L, rpm_R):
        if not GPIO_AVAILABLE: return
        GPIO.output(GPIO_REV_LEFT, GPIO.HIGH if rpm_L < 0 else GPIO.LOW)
        GPIO.output(GPIO_REV_RIGHT, GPIO.HIGH if rpm_R < 0 else GPIO.LOW)
        
        target_dac_L = int((abs(rpm_L) / WHEEL_RPM_PHYS_MAX) * DAC_MAX_VALUE)
        target_dac_R = int((abs(rpm_R) / WHEEL_RPM_PHYS_MAX) * DAC_MAX_VALUE)
        
        if abs(rpm_L) > 0.01: target_dac_L = max(1100, target_dac_L)
        if abs(rpm_R) > 0.01: target_dac_R = max(1100, target_dac_R)

        max_step = 200
        for target, last, addr in [(target_dac_L, self.last_dac_L, DAC_LEFT_ADDR), 
                                   (target_dac_R, self.last_dac_R, DAC_RIGHT_ADDR)]:
            diff = target - last
            dac = last + max_step if diff > max_step else (last - max_step if diff < -max_step else target)
            self.write_dac(addr, max(0, min(DAC_MAX_VALUE, dac)))
            if addr == DAC_LEFT_ADDR: self.last_dac_L = dac
            else: self.last_dac_R = dac

    def brake(self):
        self.write_dac(DAC_LEFT_ADDR, DAC_ZERO_VALUE)
        self.write_dac(DAC_RIGHT_ADDR, DAC_ZERO_VALUE)
        if GPIO_AVAILABLE:
            GPIO.output(GPIO_BRAKE_LEFT, GPIO.HIGH)
            GPIO.output(GPIO_BRAKE_RIGHT, GPIO.HIGH)

    def close(self):
        if self.closed: return
        self.closed = True
        self.brake()
        if GPIO_AVAILABLE: GPIO.output(GPIO_CONTACTOR, GPIO.LOW)

class BBBEncoders:
    def __init__(self):
        self.paths = {
            "L": "/sys/bus/counter/devices/counter1/count0/count",
            "R": "/sys/bus/counter/devices/counter2/count0/count"
        }
        self.prev_counts = {"L": None, "R": None}
        self.raw_counts = {"L": 0, "R": 0}
        
    def read(self):
        counts = {}
        for k, path in self.paths.items():
            try:
                with open(path, "r") as f: counts[k] = int(f.read().strip())
            except Exception:
                counts[k] = 0
        self.raw_counts = counts
        return counts["L"], counts["R"]

    def update_odometry(self):
        curr_L, curr_R = self.read()
        if self.prev_counts["L"] is None:
            self.prev_counts["L"], self.prev_counts["R"] = curr_L, curr_R
            return 0.0, 0.0

        delta_L = curr_L - self.prev_counts["L"]
        delta_R = curr_R - self.prev_counts["R"]
        
        for d_var in [delta_L, delta_R]:
            if d_var > 2**31: d_var -= 2**32
            elif d_var < -2**31: d_var += 2**32

        self.prev_counts["L"], self.prev_counts["R"] = curr_L, curr_R
        dist_L = (delta_L / (CPR * GEAR_RATIO)) * 2 * math.pi * a * LEFT_MOTOR_DIR
        dist_R = (delta_R / (CPR * GEAR_RATIO)) * 2 * math.pi * a * RIGHT_MOTOR_DIR
        return dist_L, dist_R

class BBBIMU:
    def __init__(self, bus):
        self.bus = bus
        self.gyro_scale = 131.0
        self.offset_gz = 0.0
        self.raw_gz_reading = 0.0
        
    def read_gyro_z(self):
        if not self.bus: return 0.0
        try:
            high = self.bus.read_byte_data(0x68, 0x47)
            low = self.bus.read_byte_data(0x68, 0x48)
            val = (high << 8) | low
            if val > 32767: val -= 65536
            gz = (val / self.gyro_scale) - self.offset_gz
            if abs(gz) < 0.40: gz = 0.0 
            self.raw_gz_reading = -gz # Inverted
            return self.raw_gz_reading
        except Exception:
            return 0.0

def wrap_to_pi(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

# ==========================================
# MAIN ROUTINE
# ==========================================
def main():
    # 1. Load CSV Waypoints
    csv_path = os.path.expanduser("waypoints.csv")
    try:
        data = np.loadtxt(csv_path, delimiter=",", skiprows=1)
        waypoints = data[:, 1:3]
        num_waypoints = len(waypoints)
        print(f"[Main] Loaded {num_waypoints} waypoints from {csv_path}")
    except Exception as e:
        print(f"[Error] Could not load CSV: {e}")
        sys.exit(1)

    # 2. Setup Logging
    log_dir = "/home/debian/Robot/logs"
    os.makedirs(log_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = os.path.join(log_dir, f"path_log_{timestamp}.csv")
    
    try:
        csv_file = open(log_filename, mode='w', newline='')
        csv_writer = csv.writer(csv_file)
        # Header row
        csv_writer.writerow(["time_s", "wp_target", "x_m", "y_m", "yaw_deg", 
                             "dist_err", "cmd_v", "cmd_w", "cmd_rpm_l", "cmd_rpm_r", 
                             "enc_raw_l", "enc_raw_r", "imu_gz"])
        print(f"[Main] Logging raw hardware data to: {log_filename}")
    except Exception as e:
        print(f"[Error] Could not open log file: {e}")
        csv_writer = None

    # 3. Initialize Hardware
    hw = BBBHardware()
    encoders = BBBEncoders()
    imu = BBBIMU(hw.bus)
    telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    TELEMETRY_ADDR = ("127.0.0.1", 5005)

    def sig_handler(sig, frame):
        print("\n[Main] Halting robot safely...")
        hw.close()
        if 'csv_file' in locals(): csv_file.close()
        sys.exit(0)
    signal.signal(signal.SIGINT, sig_handler)

    # 4. Set Initial Pose (from first row in CSV)
    x, y = waypoints[0, 0], waypoints[0, 1]
    yaw = 0.0 # Assuming robot starts powered on facing "0" relative to local frame
    start_time = time.time()
    last_time = start_time

    # ==========================================
    # ALIGNMENT PHASE: Rotate to face WP 1
    # ==========================================
    print("\n--- ALIGNMENT PHASE ---")
    print(f"Current Position: ({x:.2f}, {y:.2f}). Rotating to face Waypoint 1...")
    target_yaw = np.arctan2(waypoints[1, 1] - y, waypoints[1, 0] - x)
    
    while True:
        loop_start = time.time()
        dt_actual = loop_start - last_time
        last_time = loop_start
        if dt_actual <= 0 or dt_actual > 0.5: dt_actual = dt

        # Read Odometry & IMU
        dist_L, dist_R = encoders.update_odometry()
        gz = imu.read_gyro_z()
        yaw = wrap_to_pi(yaw + math.radians(gz) * dt_actual)
        
        # Calculate heading error
        yaw_err = wrap_to_pi(target_yaw - yaw)
        
        if abs(yaw_err) < align_thresh:
            print(f"Alignment Complete. Yaw locked at {math.degrees(yaw):.1f}°")
            hw.brake()
            time.sleep(0.5) # Pause briefly to settle before driving
            break
            
        # Pure rotation command (v = 0)
        v = 0.0
        w = np.clip(Kp_ang * yaw_err, -max_ang, max_ang)
        
        VL = v - d * w
        VR = v + d * w
        rpm_L = (VL / a / (2 * math.pi)) * 60.0
        rpm_R = (VR / a / (2 * math.pi)) * 60.0
        
        hw.drive_motors(rpm_L, rpm_R)
        print(f"Aligning... Yaw: {math.degrees(yaw):>5.1f}° | Target: {math.degrees(target_yaw):>5.1f}°", end='\r')
        
        elapsed = time.time() - loop_start
        if elapsed < dt: time.sleep(dt - elapsed)

    # ==========================================
    # PATH FOLLOWING PHASE
    # ==========================================
    wp_idx = 1
    print("\n\n--- PATH TRACKING PHASE ---")
    
    try:
        while wp_idx < num_waypoints:
            loop_start = time.time()
            dt_actual = loop_start - last_time
            last_time = loop_start
            if dt_actual <= 0 or dt_actual > 0.5: dt_actual = dt

            # Read Hardware & Update Pose
            dist_L, dist_R = encoders.update_odometry()
            d_center = (dist_L + dist_R) / 2.0
            gz = imu.read_gyro_z()
            yaw = wrap_to_pi(yaw + math.radians(gz) * dt_actual)
            
            x += d_center * math.cos(yaw)
            y += d_center * math.sin(yaw)

            # Waypoint Controller
            goal_x, goal_y = waypoints[wp_idx, 0], waypoints[wp_idx, 1]
            dx, dy = goal_x - x, goal_y - y
            dist = np.hypot(dx, dy)

            if dist < wp_thresh:
                print(f"Reached WP {wp_idx}/{num_waypoints-1} at ({x:.2f}, {y:.2f})")
                wp_idx += 1
                continue

            target_yaw = np.arctan2(dy, dx)
            yaw_err = wrap_to_pi(target_yaw - yaw)

            v = np.clip(Kp_lin * dist, 0.05, max_lin)
            v *= max(0.2, math.cos(yaw_err)) # Corner slowdown
            w = np.clip(Kp_ang * yaw_err, -max_ang, max_ang)

            VL = v - d * w
            VR = v + d * w
            rpm_L = (VL / a / (2 * math.pi)) * 60.0
            rpm_R = (VR / a / (2 * math.pi)) * 60.0

            # Scale to prevent physical clipping
            peak = max(abs(rpm_L), abs(rpm_R))
            if peak > WHEEL_RPM_PHYS_MAX:
                scale = WHEEL_RPM_PHYS_MAX / peak
                rpm_L *= scale
                rpm_R *= scale

            hw.drive_motors(rpm_L, rpm_R)

            # --- DATA LOGGING ---
            if csv_writer:
                csv_writer.writerow([
                    round(time.time() - start_time, 3), wp_idx, round(x, 4), round(y, 4), round(math.degrees(yaw), 2),
                    round(dist, 4), round(v, 3), round(w, 3), round(rpm_L, 1), round(rpm_R, 1),
                    encoders.raw_counts["L"], encoders.raw_counts["R"], round(imu.raw_gz_reading, 2)
                ])
                csv_file.flush() # Ensure data is written immediately in case of sudden power loss

            # --- UDP TELEMETRY ---
            try:
                tel_data = {"Xn": round(x, 3), "Yn": round(y, 3), "yaw": round(math.degrees(yaw), 1),
                            "cmd_rpm_l": round(rpm_L, 1), "cmd_rpm_r": round(rpm_R, 1)}
                telemetry_sock.sendto(json.dumps(tel_data).encode('utf-8'), TELEMETRY_ADDR)
            except Exception: pass

            print(f"Tracking WP {wp_idx} | Pose: x={x:>5.2f} y={y:>5.2f} yaw={math.degrees(yaw):>+5.1f}° | Dist: {dist:>4.2f}m", end='\r')

            elapsed = time.time() - loop_start
            if elapsed < dt: time.sleep(dt - elapsed)

        print("\n\n✅ Path Following Complete!")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        hw.close()
        if 'csv_file' in locals() and not csv_file.closed:
            csv_file.close()

if __name__ == "__main__":
    main()

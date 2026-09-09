#!/usr/bin/env python3
"""
P_Controller_log_3_edit.py
------------------------
Self-contained closed-loop CSV waypoint trajectory tracking for BeagleBone Black.
Uses BNO055 IMU in 6-DOF IMU mode (magnetic immune) and direct local GPIO/DAC interfaces.
"""

import time
from ugv_logger import UGVLogger
import math
import sys
import os
import signal
import csv
from datetime import datetime

# Try importing hardware libraries
try:
    from smbus2 import SMBus, i2c_msg
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False

try:
    import Adafruit_BBIO.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

# ==========================================
# ROBOT CONFIGURATION (Big UGV parameters)
# ==========================================
# NAVIGATION SOURCE TOGGLE
# True:  Heading (psi) uses BNO055 IMU Absolute Fused Euler angles (drift-free)
# False: Heading (psi) uses Differential Wheel Encoder Kinematics
USE_IMU_FOR_YAW = True 

a = 0.175             # Wheel radius (m)
d = 0.30              # Half wheelbase (m) - L/2
CPR = 2400            # Encoder counts per revolution (600 PPR * 4)
dt = 0.05             # Control loop rate (20 Hz matching simulation)

WHEEL_RPM_MAX = 10.0      # Command cap
WHEEL_RPM_PHYS_MAX = 20.0 # Match actual physical max wheel shaft RPM
DAC_MAX_VALUE = 4095
DAC_ZERO_VALUE = 0

# DAC & GPIO Addresses (Copied from working BBBstraightline_pid.py)
DAC_LEFT_ADDR = 0x61
DAC_RIGHT_ADDR = 0x60

GPIO_CONTACTOR = "P8_14"
GPIO_REV_LEFT = "P8_7"
GPIO_REV_RIGHT = "P8_9"
GPIO_BRAKE_LEFT = "P8_8"
GPIO_BRAKE_RIGHT = "P8_10"

# Direction Multipliers
LEFT_MOTOR_DIR = -1
RIGHT_MOTOR_DIR = 1

# Controller Parameters (From Simulation)
Kp_lin = 0.8
Kp_ang = 1.3
max_lin = 0.25
max_ang = 1.5
wp_thresh = 0.2 # 10 cm threshold

# ==========================================
# SPEED PI CONTROLLER CLASS
# ==========================================
class PIDController:
    def __init__(self, Kp, Ki, Kd, max_output, min_output, max_integral=150.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_output = max_output
        self.min_output = min_output
        self.max_integral = max_integral
        
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        if dt <= 0:
            dt = 0.05
        P = self.Kp * error
        self.integral += error * dt
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        I = self.Ki * self.integral
        D = self.Kd * (error - self.prev_error) / dt
        self.prev_error = error
        output = P + I + D
        return max(self.min_output, min(self.max_output, output))
        
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

# ==========================================
# HARDWARE INTERFACE (Contactor, Brakes, Speed Loops, DAC)
# ==========================================
class BBBHardware:
    def __init__(self):
        self.bus = SMBus(2) if HW_AVAILABLE else None
        self.is_energized = False
        self.last_dac_L = 0
        self.last_dac_R = 0
        self.closed = False
        
        # Local wheel speed PI loops (windup clamped to 150.0)
        self.pid_speed_L = PIDController(Kp=50.0, Ki=15.0, Kd=0.0, max_output=4095-1100, min_output=-250, max_integral=150.0)
        self.pid_speed_R = PIDController(Kp=50.0, Ki=15.0, Kd=0.0, max_output=4095-1100, min_output=-250, max_integral=150.0)
        
        if GPIO_AVAILABLE:
            # Ensure correct pinmux state
            for pin in [GPIO_REV_LEFT, GPIO_REV_RIGHT, GPIO_BRAKE_LEFT, GPIO_BRAKE_RIGHT, GPIO_CONTACTOR]:
                os.system(f"config-pin {pin} gpio >/dev/null 2>&1 || true")
                
            for pin in [GPIO_REV_LEFT, GPIO_REV_RIGHT, GPIO_BRAKE_LEFT, GPIO_BRAKE_RIGHT, GPIO_CONTACTOR]:
                GPIO.setup(pin, GPIO.OUT)
                
            # Initial safe state: contactor open, brakes engaged (HIGH)
            GPIO.output(GPIO_CONTACTOR, GPIO.LOW)
            GPIO.output(GPIO_REV_LEFT, GPIO.LOW)
            GPIO.output(GPIO_REV_RIGHT, GPIO.LOW)
            GPIO.output(GPIO_BRAKE_LEFT, GPIO.HIGH)
            GPIO.output(GPIO_BRAKE_RIGHT, GPIO.HIGH)
            
            self.energize()
        else:
            print("[HW] WARNING: GPIO simulation mode (GPIO library not available)")
            
    def energize(self):
        if self.is_energized: return
        print("[HW] Energizing system (closing main contactor)...")
        if GPIO_AVAILABLE:
            GPIO.output(GPIO_CONTACTOR, GPIO.HIGH)
            time.sleep(0.5)
            print("[HW] Releasing mechanical brakes...")
            GPIO.output(GPIO_BRAKE_LEFT, GPIO.LOW)
            GPIO.output(GPIO_BRAKE_RIGHT, GPIO.LOW)
            time.sleep(0.1)
        self.is_energized = True
        
    def deenergize(self):
        if not self.is_energized: return
        print("[HW] De-energizing system (opening main contactor)...")
        if GPIO_AVAILABLE:
            GPIO.output(GPIO_CONTACTOR, GPIO.LOW)
        self.is_energized = False

    def write_dac(self, addr, value):
        if not self.bus: return
        value = max(0, min(value, DAC_MAX_VALUE))
        upper = (value >> 4) & 0xFF
        lower = (value << 4) & 0xFF
        try:
            self.bus.write_i2c_block_data(addr, 0x40, [upper, lower])
        except Exception as e:
            print(f"[HW] I2C write error on {hex(addr)}: {e}")

    def set_gpios(self, rev_L, rev_R, brake_on=False):
        if not GPIO_AVAILABLE: return
        GPIO.output(GPIO_REV_LEFT, GPIO.HIGH if rev_L else GPIO.LOW)
        GPIO.output(GPIO_REV_RIGHT, GPIO.HIGH if rev_R else GPIO.LOW)
        GPIO.output(GPIO_BRAKE_LEFT, GPIO.HIGH if brake_on else GPIO.LOW)
        GPIO.output(GPIO_BRAKE_RIGHT, GPIO.HIGH if brake_on else GPIO.LOW)

    def drive_motors(self, rpm_L, rpm_R, enc_l=None, enc_r=None, dt=0.05):
        # Set relay directions (LOW for forward, HIGH for reverse)
        self.set_gpios(rpm_L < 0, rpm_R < 0, brake_on=False)
        
        if enc_l is not None and enc_r is not None:
            # Closed-loop speed PI calculation
            target_L = abs(rpm_L)
            target_R = abs(rpm_R)
            meas_L = abs(enc_l)
            meas_R = abs(enc_r)
            
            # Left wheel DAC
            if target_L < 0.01:
                target_dac_L = 0
                self.pid_speed_L.reset()
            else:
                err_L = target_L - meas_L
                target_dac_L = 1100 + int(self.pid_speed_L.update(err_L, dt))
                
            # Right wheel DAC
            if target_R < 0.01:
                target_dac_R = 0
                self.pid_speed_R.reset()
            else:
                err_R = target_R - meas_R
                target_dac_R = 1100 + int(self.pid_speed_R.update(err_R, dt))
        else:
            # Open-loop fallback mapping
            target_dac_L = int((abs(rpm_L) / WHEEL_RPM_PHYS_MAX) * DAC_MAX_VALUE)
            target_dac_R = int((abs(rpm_R) / WHEEL_RPM_PHYS_MAX) * DAC_MAX_VALUE)
            if abs(rpm_L) > 0.01: target_dac_L = max(1100, target_dac_L)
            if abs(rpm_R) > 0.01: target_dac_R = max(1100, target_dac_R)

        target_dac_L = max(0, min(DAC_MAX_VALUE, target_dac_L))
        target_dac_R = max(0, min(DAC_MAX_VALUE, target_dac_R))
        
        # Apply slew rate limit
        max_step = 200
        
        # Left DAC ramping
        diff_L = target_dac_L - self.last_dac_L
        if diff_L > max_step:
            dac_L = self.last_dac_L + max_step
        elif diff_L < -max_step:
            dac_L = self.last_dac_L - max_step
        else:
            dac_L = target_dac_L
            
        # Right DAC ramping
        diff_R = target_dac_R - self.last_dac_R
        if diff_R > max_step:
            dac_R = self.last_dac_R + max_step
        elif diff_R < -max_step:
            dac_R = self.last_dac_R - max_step
        else:
            dac_R = target_dac_R
            
        self.last_dac_L = dac_L
        self.last_dac_R = dac_R
        
        self.write_dac(DAC_LEFT_ADDR, dac_L)
        self.write_dac(DAC_RIGHT_ADDR, dac_R)

    def brake(self):
        self.write_dac(DAC_LEFT_ADDR, DAC_ZERO_VALUE)
        self.write_dac(DAC_RIGHT_ADDR, DAC_ZERO_VALUE)
        self.set_gpios(False, False, brake_on=True)

    def close(self):
        if self.closed: return
        self.closed = True
        self.brake()
        self.deenergize()
        if self.bus: self.bus.close()
        # Safe state: set all safety pins to LOW, but do NOT cleanup() 
        # to retain sysfs export permissions for subsequent reruns.
        if GPIO_AVAILABLE:
            for pin in [GPIO_REV_LEFT, GPIO_REV_RIGHT, GPIO_BRAKE_LEFT, GPIO_BRAKE_RIGHT, GPIO_CONTACTOR]:
                try: GPIO.output(pin, GPIO.LOW)
                except Exception: pass

# ==========================================
# E-QEP ENCODER INTERFACE
# ==========================================
class BBBEncoders:
    def __init__(self):
        self.paths = {
            "L": "/sys/bus/counter/devices/counter1/count0/count",
            "R": "/sys/bus/counter/devices/counter2/count0/count"
        }
        self.prev_counts = {"L": None, "R": None}
        
        for dev in ["counter1", "counter2"]:
            ceiling_path = f"/sys/bus/counter/devices/{dev}/count0/ceiling"
            enable_path = f"/sys/bus/counter/devices/{dev}/count0/enable"
            if os.path.exists(ceiling_path):
                try:
                    with open(ceiling_path, "w") as f: f.write("4294967295")
                except Exception: pass
            if os.path.exists(enable_path):
                try:
                    with open(enable_path, "w") as f: f.write("1")
                except Exception: pass
                    
    def read(self):
        counts = {}
        for k, path in self.paths.items():
            try:
                with open(path, "r") as f: counts[k] = int(f.read().strip())
            except Exception:
                counts[k] = 0
        return counts["L"], counts["R"]

    def update_odometry(self):
        curr_L, curr_R = self.read()
        if self.prev_counts["L"] is None:
            self.prev_counts["L"], self.prev_counts["R"] = curr_L, curr_R
            return 0.0, 0.0, curr_L, curr_R

        delta_L = curr_L - self.prev_counts["L"]
        delta_R = curr_R - self.prev_counts["R"]
        
        for d_val in ("L", "R"):
            d_var = delta_L if d_val == "L" else delta_R
            if d_var > 2**31: d_var -= 2**32
            elif d_var < -2**31: d_var += 2**32
            if d_val == "L": delta_L = d_var
            else: delta_R = d_var

        self.prev_counts["L"], self.prev_counts["R"] = curr_L, curr_R

        dist_L = (delta_L / CPR) * 2 * math.pi * a * LEFT_MOTOR_DIR
        dist_R = (delta_R / CPR) * 2 * math.pi * a * RIGHT_MOTOR_DIR

        return dist_L, dist_R, curr_L, curr_R

# ==========================================
# BNO055 IMU INTERFACE (9-DOF)
# ==========================================
class BBBIMU:
    def __init__(self, bus):
        self.bus = bus
        self.initialized = False
        self.sensor = None
        try:
            import board
            import adafruit_bno055
            i2c = board.I2C()
            self.sensor = adafruit_bno055.BNO055_I2C(i2c)
            # Set mode to 6-DOF IMU mode (0x08) to run magnetic-immune
            self.sensor.mode = 0x08
            self.initialized = True
            print("[IMU] BNO055 IMU initialized in 6-DOF IMU mode.")
            self.load_calibration()
        except Exception as e:
            print(f"[IMU] Warning: Failed to initialize BNO055: {e}")

    def load_calibration(self):
        import json
        cal_file = "/home/debian/Robot/bno055_calibration.json"
        if not self.initialized or self.sensor is None: return
        if not os.path.exists(cal_file): return
        try:
            with open(cal_file, "r") as f:
                offsets = json.load(f)
            self.sensor.offsets_accelerometer = tuple(offsets["accel"])
            self.sensor.offsets_gyroscope     = tuple(offsets["gyro"])
            self.sensor.offsets_magnetometer  = tuple(offsets["mag"])
            if "accel_radius" in offsets: self.sensor.radius_accelerometer = offsets["accel_radius"]
            if "mag_radius" in offsets: self.sensor.radius_magnetometer  = offsets["mag_radius"]
            print(f"[IMU] Calibration offsets restored from {cal_file}")
        except Exception as e:
            print(f"[IMU] Warning: Could not restore calibration: {e}")

    def read_absolute_yaw(self):
        """Read absolute orientation yaw in radians."""
        if not self.initialized or self.sensor is None:
            return None
        try:
            euler = self.sensor.euler
            if euler is None or euler[0] is None:
                return None
            yaw_deg = euler[0]
            return math.radians(yaw_deg)
        except Exception:
            return None

    def read_all(self):
        """Returns 9-DOF data: ax, ay, az, gx, gy, gz, mx, my, mz"""
        if not self.initialized or self.sensor is None:
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            
        try:
            accel = self.sensor.acceleration
            gyro = self.sensor.gyro
            mag = self.sensor.magnetic
            
            ax, ay, az = [val / 9.80665 if val else 0.0 for val in (accel or (0,0,0))]
            gx, gy, gz = [val * 57.29577951308232 if val else 0.0 for val in (gyro or (0,0,0))]
            mx, my, mz = [val if val else 0.0 for val in (mag or (0,0,0))]
            
            return ax, ay, az, gx, gy, -gz, mx, my, mz
        except Exception:
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

# ==========================================
# HELPER FUNCTIONS
# ==========================================
def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def load_waypoints(csv_path):
    waypoints = []
    try:
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                waypoints.append((float(row[1]), float(row[2])))
    except Exception as e:
        print(f"Error loading waypoints: {e}")
        sys.exit(1)
    return waypoints

# ==========================================
# MAIN CLOSED-LOOP CONTROLLER
# ==========================================
def main():
    logger = UGVLogger()
    csv_path = "/home/debian/Robot/waypoints_1m.csv"
    waypoints = load_waypoints(csv_path)
    num_waypoints = len(waypoints)
    print(f"Loaded {num_waypoints} waypoints.")
    
    if num_waypoints < 2:
        print("Need at least 2 waypoints to navigate.")
        return

    hw = BBBHardware()
    
    def sig_handler(sig, frame):
        print(f"\n[Main] Signal {sig} received. Stopping safely...")
        hw.close()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)
    
    encoders = BBBEncoders()
    imu = BBBIMU(hw.bus)
    


    # Initialize Pose (Starting at WP 0, facing WP 1)
    x, y = waypoints[0]
    dx0 = waypoints[1][0] - waypoints[0][0]
    dy0 = waypoints[1][1] - waypoints[0][1]
    psi = math.atan2(dy0, dx0)
    
    yaw_start = None
    wp_idx = 1
    rpm_L, rpm_R = 0.0, 0.0
    start_time = time.time()
    last_time = start_time
    
    nav_mode_str = "IMU (Absolute Fused Euler)" if USE_IMU_FOR_YAW else "Encoders (Differential Math)"
    print(f"Starting Navigation Loop (dt={dt}s) | Nav Mode: {nav_mode_str}")

    try:
        while wp_idx < num_waypoints:
            loop_start = time.time()
            dt_actual = loop_start - last_time
            last_time = loop_start
            delta_L = 0
            delta_R = 0
            
            if dt_actual <= 0 or dt_actual > 0.5: dt_actual = dt
            
            # 1. --- Update Odometry & Distance ---
            prev_L_cnt, prev_R_cnt = encoders.prev_counts["L"], encoders.prev_counts["R"]
            dist_L, dist_R, curr_L_cnt, curr_R_cnt = encoders.update_odometry()
            d_center = (dist_L + dist_R) / 2.0
            
            # 2. --- Calculate Encoder RPMs ---
            enc_rpm_l, enc_rpm_r = 0.0, 0.0
            if prev_L_cnt is not None:
                delta_L = curr_L_cnt - prev_L_cnt
                if delta_L > 2**31: delta_L -= 2**32
                elif delta_L < -2**31: delta_L += 2**32
                enc_rpm_l = (delta_L / (CPR * dt_actual)) * 60.0 * LEFT_MOTOR_DIR
                
            if prev_R_cnt is not None:
                delta_R = curr_R_cnt - prev_R_cnt
                if delta_R > 2**31: delta_R -= 2**32
                elif delta_R < -2**31: delta_R += 2**32
                enc_rpm_r = (delta_R / (CPR * dt_actual)) * 60.0 * RIGHT_MOTOR_DIR

            # 3. --- Calculate Angular Velocity from Encoders (w_rad/s) ---
            V_L_enc = (enc_rpm_l * (2 * math.pi) / 60.0) * a
            V_R_enc = (enc_rpm_r * (2 * math.pi) / 60.0) * a
            enc_w_rads = (V_R_enc - V_L_enc) / (2.0 * d)

            # 4. --- Read IMU ---
            ax, ay, az, gx, gy, gz, mx, my, mz = imu.read_all()
            
            # 5. --- Update Heading (Toggleable) ---
            if USE_IMU_FOR_YAW:
                yaw_raw = imu.read_absolute_yaw()
                if yaw_raw is not None:
                    yaw_actual = -yaw_raw
                    if yaw_start is None:
                        yaw_start = yaw_actual
                        print(f"[IMU] Absolute Fused Yaw Lock: start={math.degrees(yaw_start):.1f}°")
                    psi = yaw_actual - yaw_start
            else:
                psi += enc_w_rads * dt_actual
                
            psi = wrap_to_pi(psi)
            
            # 6. --- Update X, Y Coordinate ---
            x += d_center * math.cos(psi)
            y += d_center * math.sin(psi)
            
            # --- Log Step (writes CSV and broadcasts UDP telemetry automatically) ---
            logger.log_step(
                dt_actual=dt_actual,
                x_fused=x, y_fused=y, yaw_deg=math.degrees(psi),
                cmd_l=rpm_L, cmd_r=rpm_R,
                enc_l=enc_rpm_l, enc_r=enc_rpm_r,
                raw_l=curr_L_cnt if curr_L_cnt is not None else 0,
                raw_r=curr_R_cnt if curr_R_cnt is not None else 0,
                delta_l=delta_L, delta_r=delta_R,
                ax=ax, ay=ay, az=az,
                gx=gx, gy=gy, gz=gz,
                mx=mx, my=my, mz=mz,
                x_imu=0.0, y_imu=0.0
            )

            # --- Waypoint Controller Logic ---
            goal_x, goal_y = waypoints[wp_idx]
            dist_to_goal = math.hypot(goal_x - x, goal_y - y)
            
            if dist_to_goal < wp_thresh:
                print(f"\nReached WP {wp_idx}/{num_waypoints-1}")
                wp_idx += 1
                if wp_idx >= num_waypoints:
                    break
                continue
                
            target_yaw = math.atan2(goal_y - y, goal_x - x)
            yaw_err = wrap_to_pi(target_yaw - psi)
            
            # Smooth Controller
            V = min(Kp_lin * dist_to_goal, max_lin)
            V = max(0.05, V)
            V *= max(0.2, math.cos(yaw_err))
            
            W = max(-max_ang, min(Kp_ang * yaw_err, max_ang))
            
            # Inverse Kinematics
            VL = V - d * W
            VR = V + d * W
            
            # Convert to RPM
            rpm_L = ((VL / a) / (2 * math.pi)) * 60.0
            rpm_R = ((VR / a) / (2 * math.pi)) * 60.0
            
            # Cap RPMs proportionally
            peak = max(abs(rpm_L), abs(rpm_R))
            if peak > WHEEL_RPM_MAX:
                scale = WHEEL_RPM_MAX / peak
                rpm_L *= scale
                rpm_R *= scale
                
            # 7. --- Commiting Motor Speeds (Closed-Loop) ---
            hw.drive_motors(rpm_L, rpm_R, enc_l=enc_rpm_l, enc_r=enc_rpm_r, dt=dt_actual)
            
            print(f"WP:{wp_idx} | Pose: x={x:>5.2f} y={y:>5.2f} yaw={math.degrees(psi):>+6.1f}° | Err: {dist_to_goal:.2f}m | Cmd: {rpm_L:+.1f},{rpm_R:+.1f}", end='\r')
            
            # Timing
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
                
        print("\n\n✅ Path Following Complete. All waypoints reached.")

    except KeyboardInterrupt:
        print("\nInterrupted! Stopping robot.")
    finally:
        hw.close()
        logger.close()

if __name__ == "__main__":
    main()

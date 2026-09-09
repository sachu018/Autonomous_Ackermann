#!/usr/bin/env python3
"""
teleop_key.py
-------------
Keyboard teleoperation script for the UGV over SSH.
Features a 300ms dead-man's watchdog, dry-switching relay protection,
and integrates the modular ugv_logger for automatic telemetry logging.
"""

import time
import math
import sys
import os
import select
import tty
import termios
from datetime import datetime

# Import modular UGV logger
from ugv_logger import UGVLogger

# Try importing hardware libraries
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
# ROBOT CONFIGURATION (Big UGV parameters)
# ==========================================
a = 0.175             # Wheel radius (m)
d = 0.30              # Half wheelbase (m)
CPR = 2400            # Encoder counts per revolution (600 PPR * 4)
dt = 0.1              # Control loop rate (10 Hz)

WHEEL_RPM_MAX = 10.0      # Command cap
WHEEL_RPM_PHYS_MAX = 20.0 # Physical maximum
DAC_MAX_VALUE = 4095
DAC_ZERO_VALUE = 0

# DAC & GPIO Addresses
DAC_LEFT_ADDR = 0x61
DAC_RIGHT_ADDR = 0x60

GPIO_CONTACTOR = "P8_14"
GPIO_REV_LEFT = "P8_7"
GPIO_REV_RIGHT = "P8_9"
GPIO_BRAKE_LEFT = "P8_8"
GPIO_BRAKE_RIGHT = "P8_10"

LEFT_MOTOR_DIR = -1
RIGHT_MOTOR_DIR = 1

# Teleop Target Speeds
SPEED_LINEAR = 0.15   # 0.15 m/s forward/reverse crawl
SPEED_ANGULAR = 0.6   # 0.6 rad/s pivot turn rate

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
        if dt <= 0: dt = 0.1
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
# HARDWARE INTERFACE (Dry-switching relays, Speed Loops, DAC)
# ==========================================
class BBBHardware:
    def __init__(self):
        self.bus = SMBus(2) if HW_AVAILABLE else None
        self.is_energized = False
        self.last_dac_L = 0
        self.last_dac_R = 0
        self.closed = False
        
        self.pid_speed_L = PIDController(Kp=50.0, Ki=15.0, Kd=0.0, max_output=4095-1100, min_output=-250, max_integral=150.0)
        self.pid_speed_R = PIDController(Kp=50.0, Ki=15.0, Kd=0.0, max_output=4095-1100, min_output=-250, max_integral=150.0)
        
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
        else:
            print("[HW] WARNING: GPIO simulation mode.")
            
    def energize(self):
        if self.is_energized: return
        print("[HW] Energizing UGV safety contactor...")
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
        print("[HW] De-energizing UGV safety contactor...")
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
            # We don't print inside the fast loop to avoid console clutter over SSH
            pass

    def set_gpios(self, rev_L, rev_R, brake_on=False):
        if not GPIO_AVAILABLE: return
        GPIO.output(GPIO_REV_LEFT, GPIO.HIGH if rev_L else GPIO.LOW)
        GPIO.output(GPIO_REV_RIGHT, GPIO.HIGH if rev_R else GPIO.LOW)
        GPIO.output(GPIO_BRAKE_LEFT, GPIO.HIGH if brake_on else GPIO.LOW)
        GPIO.output(GPIO_BRAKE_RIGHT, GPIO.HIGH if brake_on else GPIO.LOW)

    def drive_motors(self, rpm_L, rpm_R, enc_l=None, enc_r=None, dt=0.1):
        new_rev_L = rpm_L < 0
        new_rev_R = rpm_R < 0
        
        if not hasattr(self, 'current_rev_L'):
            self.current_rev_L = False
            self.current_rev_R = False
            
        # Dry-Switching Relay Protection
        if new_rev_L != self.current_rev_L or new_rev_R != self.current_rev_R:
            self.write_dac(DAC_LEFT_ADDR, 0)
            self.write_dac(DAC_RIGHT_ADDR, 0)
            self.last_dac_L = 0
            self.last_dac_R = 0
            time.sleep(0.15)
            self.set_gpios(new_rev_L, new_rev_R, brake_on=False)
            self.current_rev_L = new_rev_L
            self.current_rev_R = new_rev_R
            time.sleep(0.05)
            self.pid_speed_L.reset()
            self.pid_speed_R.reset()
        else:
            self.set_gpios(new_rev_L, new_rev_R, brake_on=False)
        
        if enc_l is not None and enc_r is not None:
            target_L = abs(rpm_L)
            target_R = abs(rpm_R)
            meas_L = abs(enc_l)
            meas_R = abs(enc_r)
            
            if target_L < 0.01:
                target_dac_L = 0
                self.pid_speed_L.reset()
            else:
                err_L = target_L - meas_L
                target_dac_L = 1100 + int(self.pid_speed_L.update(err_L, dt))
                
            if target_R < 0.01:
                target_dac_R = 0
                self.pid_speed_R.reset()
            else:
                err_R = target_R - meas_R
                target_dac_R = 1100 + int(self.pid_speed_R.update(err_R, dt))
        else:
            target_dac_L = int((abs(rpm_L) / WHEEL_RPM_PHYS_MAX) * DAC_MAX_VALUE)
            target_dac_R = int((abs(rpm_R) / WHEEL_RPM_PHYS_MAX) * DAC_MAX_VALUE)
            if abs(rpm_L) > 0.01: target_dac_L = max(1100, target_dac_L)
            if abs(rpm_R) > 0.01: target_dac_R = max(1100, target_dac_R)

        target_dac_L = max(0, min(DAC_MAX_VALUE, target_dac_L))
        target_dac_R = max(0, min(DAC_MAX_VALUE, target_dac_R))
        
        max_step = 200
        diff_L = target_dac_L - self.last_dac_L
        if diff_L > max_step: dac_L = self.last_dac_L + max_step
        elif diff_L < -max_step: dac_L = self.last_dac_L - max_step
        else: dac_L = target_dac_L
            
        diff_R = target_dac_R - self.last_dac_R
        if diff_R > max_step: dac_R = self.last_dac_R + max_step
        elif diff_R < -max_step: dac_R = self.last_dac_R - max_step
        else: dac_R = target_dac_R
            
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
        if not self.initialized or self.sensor is None: return None
        try:
            euler = self.sensor.euler
            if euler is None or euler[0] is None: return None
            return math.radians(euler[0])
        except Exception:
            return None

    def read_all(self):
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

def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

# ==========================================
# MAIN TELEOPERATION LOOP
# ==========================================
def main():
    # 1. Initialize BNO055 and calibrate accelerometer zero-biases BEFORE energizing hardware
    # This prevents the motor driver's safety watchdog from timing out during calibration sleep!
    imu = BBBIMU(None)
    
    x_imu, y_imu = 0.0, 0.0
    vel_x_imu, vel_y_imu = 0.0, 0.0
    
    if imu.initialized:
        print("[IMU] Calibrating accelerometer zero-biases... Keep UGV stationary.")
        bias_ax_sum, bias_ay_sum = 0.0, 0.0
        for _ in range(20):
            ax_raw, ay_raw, _, _, _, _, _, _, _ = imu.read_all()
            bias_ax_sum += ax_raw
            bias_ay_sum += ay_raw
            time.sleep(0.05)
        bias_ax = bias_ax_sum / 20.0
        bias_ay = bias_ay_sum / 20.0
        print(f"[IMU] Accel Bias Calibrated: Ax={bias_ax:+.4f} G, Ay={bias_ay:+.4f} G")
    else:
        print("[IMU] Warning: IMU not initialized. Bias calibration skipped.")
        bias_ax, bias_ay = 0.0, 0.0

    # 2. Now energize hardware and start encoders
    hw = BBBHardware()
    encoders = BBBEncoders()
    
    # 3. Initialize Modular UGV Logger
    logger = UGVLogger()
    
    # Save original terminal settings for raw key polling
    orig_settings = termios.tcgetattr(sys.stdin)
    
    x, y, psi = 0.0, 0.0, 0.0
    yaw_start = None
    rpm_L, rpm_R = 0.0, 0.0
    
    # Temperature polling throttle
    last_temp_time = 0.0
    temp_val = 0.0
    
    # Teleop targets & Dynamic Speeds
    V, W = 0.0, 0.0
    speed_linear = SPEED_LINEAR
    speed_angular = SPEED_ANGULAR
    
    last_key_time = time.time()
    start_time = last_key_time
    last_loop_time = last_key_time
    
    print("\n" + "=" * 60)
    print("       UGV ROS2-STYLE KEYBOARD TELEOPERATION (SAFE MODE)")
    print("=" * 60)
    print("  Directional Control (Hold down key to drive):")
    print("        u   i   o      (Forward-Left  /  Forward  /  Forward-Right)")
    print("        j   k   l      (Pivot-Left    /   Stop    /  Pivot-Right)")
    print("        m   ,   .      (Reverse-Left  /  Reverse  /  Reverse-Right)")
    print("  ")
    print("  Speed Adjustments (Tap to change by +/- 10%):")
    print("    q / z : Increase / decrease max linear speed")
    print("    w / x : Increase / decrease max angular speed")
    print("  ")
    print("  Exit: Press Ctrl+C to quit safely")
    print("  [SAFETY]: Watchdog will stop UGV if no key is pressed for 300ms.")
    print("=" * 60 + "\n")

    try:
        # Put terminal in raw mode
        tty.setraw(sys.stdin.fileno())
        
        while True:
            loop_start = time.time()
            dt_actual = loop_start - last_loop_time
            last_loop_time = loop_start
            
            if dt_actual <= 0 or dt_actual > 0.5: dt_actual = dt
            
            # --- Drain/Flush stdin buffer to read ONLY the latest keystroke ---
            key = None
            while True:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
                if rlist:
                    char = sys.stdin.read(1)
                    if char:
                        key = char
                else:
                    break
                    
            if key is not None:
                # Capture Ctrl+C (character code \x03)
                if key == '\x03':
                    print("\n[Main] Ctrl+C pressed. Exiting...")
                    break
                
                # Speed Adjustments (Tap keys)
                elif key == 'q':
                    speed_linear = round(speed_linear * 1.10, 4)
                    sys.stdout.write(f"\n[Speed adjusted] Max Linear Speed: {speed_linear:.3f} m/s\n")
                    sys.stdout.flush()
                elif key == 'z':
                    speed_linear = round(speed_linear * 0.90, 4)
                    sys.stdout.write(f"\n[Speed adjusted] Max Linear Speed: {speed_linear:.3f} m/s\n")
                    sys.stdout.flush()
                elif key == 'w':
                    speed_angular = round(speed_angular * 1.10, 4)
                    sys.stdout.write(f"\n[Speed adjusted] Max Angular Speed: {speed_angular:.3f} rad/s\n")
                    sys.stdout.flush()
                elif key == 'x':
                    speed_angular = round(speed_angular * 0.90, 4)
                    sys.stdout.write(f"\n[Speed adjusted] Max Angular Speed: {speed_angular:.3f} rad/s\n")
                    sys.stdout.flush()
                
                # Directional Controls (Hold keys)
                else:
                    last_key_time = loop_start
                    if key == 'i':
                        V = speed_linear
                        W = 0.0
                    elif key == ',':
                        V = -speed_linear
                        W = 0.0
                    elif key == 'j':
                        V = 0.0
                        W = speed_angular
                    elif key == 'l':
                        V = 0.0
                        W = -speed_angular
                    elif key == 'u':
                        V = speed_linear
                        W = speed_angular
                    elif key == 'o':
                        V = speed_linear
                        W = -speed_angular
                    elif key == 'm':
                        V = -speed_linear
                        W = speed_angular
                    elif key == '.':
                        V = -speed_linear
                        W = -speed_angular
                    elif key in ('k', 's', ' '):
                        V = 0.0
                        W = 0.0
            
            # --- Dead-man's Safety Watchdog ---
            # If no keystroke is received within 300ms, zero the velocity targets
            if loop_start - last_key_time > 0.30:
                V = 0.0
                W = 0.0

            # --- Update Odometry ---
            prev_L_cnt, prev_R_cnt = encoders.prev_counts["L"], encoders.prev_counts["R"]
            dist_L, dist_R, curr_L_cnt, curr_R_cnt = encoders.update_odometry()
            d_center = (dist_L + dist_R) / 2.0
            
            # Calculate delta ticks
            delta_L = 0
            delta_R = 0
            if prev_L_cnt is not None:
                delta_L = curr_L_cnt - prev_L_cnt
                if delta_L > 2**31: delta_L -= 2**32
                elif delta_L < -2**31: delta_L += 2**32
                
            if prev_R_cnt is not None:
                delta_R = curr_R_cnt - prev_R_cnt
                if delta_R > 2**31: delta_R -= 2**32
                elif delta_R < -2**31: delta_R += 2**32
                
            # Calculate speeds
            enc_rpm_l = (delta_L / (CPR * dt_actual)) * 60.0 * LEFT_MOTOR_DIR
            enc_rpm_r = (delta_R / (CPR * dt_actual)) * 60.0 * RIGHT_MOTOR_DIR
            
            V_L_enc = (enc_rpm_l * (2 * math.pi) / 60.0) * a
            V_R_enc = (enc_rpm_r * (2 * math.pi) / 60.0) * a
            enc_w_rads = (V_R_enc - V_L_enc) / (2.0 * d)
            
            # Read IMU sensors
            ax, ay, az, gx, gy, gz, mx, my, mz = imu.read_all()
            
            yaw_raw = imu.read_absolute_yaw()
            if yaw_raw is not None:
                yaw_actual = -yaw_raw
                if yaw_start is None:
                    yaw_start = yaw_actual
                psi = yaw_actual - yaw_start
            else:
                psi += enc_w_rads * dt_actual
            psi = wrap_to_pi(psi)
            
            # Update position coordinate
            x += d_center * math.cos(psi)
            y += d_center * math.sin(psi)
            
            # Update IMU dead-reckoning baseline
            ax_ms2 = (ax - bias_ax) * 9.80665
            ay_ms2 = (ay - bias_ay) * 9.80665
            ax_global = ax_ms2 * math.cos(psi) - ay_ms2 * math.sin(psi)
            ay_global = ax_ms2 * math.sin(psi) + ay_ms2 * math.cos(psi)
            
            vel_x_imu += ax_global * dt_actual
            vel_y_imu += ay_global * dt_actual
            x_imu += vel_x_imu * dt_actual
            y_imu += vel_y_imu * dt_actual
            
            # --- Write to Modular Logger (1 Hz temperature query) ---
            if (loop_start - last_temp_time) >= 1.0:
                last_temp_time = loop_start
                try:
                    temp_val = imu.sensor.temperature if (imu.initialized and imu.sensor is not None) else 0.0
                    if temp_val is None: temp_val = 0.0
                except Exception:
                    pass

            logger.log_step(
                dt_actual=dt_actual,
                x_fused=x, y_fused=y, yaw_deg=math.degrees(psi),
                cmd_l=rpm_L, cmd_r=rpm_R,
                enc_l=enc_rpm_l, enc_r=enc_rpm_r,
                raw_l=curr_L_cnt, raw_r=curr_R_cnt,
                delta_l=delta_L, delta_r=delta_R,
                ax=ax, ay=ay, az=az,
                gx=gx, gy=gy, gz=gz,
                mx=mx, my=my, mz=mz,
                x_imu=x_imu, y_imu=y_imu,
                temp=temp_val
            )
            
            # --- Calculate target wheel speeds ---
            # Inverse Kinematics
            VL = V - d * W
            VR = V + d * W
            
            rpm_L = ((VL / a) / (2 * math.pi)) * 60.0
            rpm_R = ((VR / a) / (2 * math.pi)) * 60.0
            
            peak = max(abs(rpm_L), abs(rpm_R))
            if peak > WHEEL_RPM_MAX:
                scale = WHEEL_RPM_MAX / peak
                rpm_L *= scale
                rpm_R *= scale
                
            # --- Output speed commands ---
            hw.drive_motors(rpm_L, rpm_R, enc_l=enc_rpm_l, enc_r=enc_rpm_r, dt=dt_actual)
            
            # Print state (returns carriage to avoid flooding terminal screen)
            # Shows active command and watchdog status
            wd_state = "DRIVING" if (loop_start - last_key_time < 0.3) else "TIMEOUT"
            sys.stdout.write(
                f"\rState: [{wd_state:^7}] | Cmd: V={V:+.2f} W={W:+.2f} | Pose: x={x:>5.2f} y={y:>5.2f} yaw={math.degrees(psi):>+6.1f}°"
            )
            sys.stdout.flush()
            
            # Loop execution frequency timing
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
                
    finally:
        # Restore original terminal settings immediately
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
        print("\n\nExiting teleoperation safely...")
        hw.close()
        logger.close()

if __name__ == "__main__":
    main()

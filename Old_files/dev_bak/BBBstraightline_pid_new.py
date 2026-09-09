#!/usr/bin/env python3
"""
BBBstraightline.py
------------------
BeagleBone Black version of closed-loop straight line trajectory tracking.
Adapted from small UGV RoboClaw kinematics to the big UGV hardware interfaces:
  - Encoders: Hardware eQEP1 and eQEP2b counters (read via sysfs)
  - Motors: I2C DACs (MCP4725) and GPIO direction/brake control
"""

import time
from ugv_logger import UGVLogger
import math
import sys
import os
import signal

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
a = 0.175             # Wheel radius (m)
d = 0.30              # Half wheelbase (m) - L/2
CPR = 2400            # Encoder counts per revolution (600 PPR * 4)
GEAR_RATIO = 20.0     # Gear ratio (1:20)
dt = 0.1              # Control loop rate (10 Hz)

WHEEL_RPM_MAX = 10.0       # Command cap: 20% of physical limit (20.0 RPM)
WHEEL_RPM_PHYS_MAX = 20.0 # Match actual physical max wheel shaft RPM
DAC_MAX_VALUE = 4095
DAC_ZERO_VALUE = 0

# DAC & GPIO Addresses (Big UGV)
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

# Controller Gain Parameters (Straight Line Tracking)
Kp_rho = 0.8          # Linear velocity proportional gain
Kp_alpha = 1.5        # Heading alignment proportional gain
MAX_LINEAR_SPEED = 0.183  # Capped at 20% of physical maximum (0.366 m/s)

# ==========================================
# HARDWARE INTERFACES
# ==========================================
class BBBHardware:
    def __init__(self):
        self.bus = SMBus(2) if HW_AVAILABLE else None
        self.is_energized = False
        self.last_dac_L = 0
        self.last_dac_R = 0
        # Initialize inner-loop wheel speed PI controllers (output controls the DAC throttle from 0-4095)
        # Tuned to Kp=50.0, Ki=15.0, Kd=0.0. max_integral=500.0.
        # min_output=-150 ensures the total DAC (1100 + PID_out) never drops below 950, keeping motors moving.
        self.pid_speed_L = PIDController(Kp=50.0, Ki=15.0, Kd=0.0, max_output=4095-1100, min_output=-250, max_integral=150.0)
        self.pid_speed_R = PIDController(Kp=50.0, Ki=15.0, Kd=0.0, max_output=4095-1100, min_output=-250, max_integral=150.0)
        self.closed = False
        
        if GPIO_AVAILABLE:
            # Automatically configure pinmux for the GPIO pins to ensure they are active
            for pin in [GPIO_REV_LEFT, GPIO_REV_RIGHT,
                        GPIO_BRAKE_LEFT, GPIO_BRAKE_RIGHT,
                        GPIO_CONTACTOR]:
                os.system(f"config-pin {pin} gpio >/dev/null 2>&1 || true")
                
            for pin in [GPIO_REV_LEFT, GPIO_REV_RIGHT,
                        GPIO_BRAKE_LEFT, GPIO_BRAKE_RIGHT,
                        GPIO_CONTACTOR]:
                GPIO.setup(pin, GPIO.OUT)
            
            # Initial safe state: contactor open, directions LOW, brakes HIGH (engaged)
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
            print("[HW] Releasing brakes...")
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

    def drive_motors(self, rpm_L, rpm_R, enc_l=None, enc_r=None, dt=0.1):
        # Write direction GPIOs (LOW for forward, HIGH for reverse)
        self.set_gpios(rpm_L < 0, rpm_R < 0, brake_on=False)
        
        if enc_l is not None and enc_r is not None:
            # Closed-loop speed PI controller
            target_L = abs(rpm_L)
            target_R = abs(rpm_R)
            meas_L = abs(enc_l)
            meas_R = abs(enc_r)
            
            # Left wheel DAC target
            if target_L < 0.01:
                target_dac_L = 0
                self.pid_speed_L.reset()
            else:
                err_L = target_L - meas_L
                target_dac_L = 1100 + int(self.pid_speed_L.update(err_L, dt))
                
            # Right wheel DAC target
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
            if abs(rpm_L) > 0.01:
                target_dac_L = max(1100, target_dac_L)
            if abs(rpm_R) > 0.01:
                target_dac_R = max(1100, target_dac_R)

        target_dac_L = max(0, min(DAC_MAX_VALUE, target_dac_L))
        target_dac_R = max(0, min(DAC_MAX_VALUE, target_dac_R))
        
        # Apply slew rate limit (max step of 200 per loop iteration of 0.1s)
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
        if GPIO_AVAILABLE: GPIO.cleanup()

# ==========================================
# E-QEP ENCODER INTERFACE (sysfs)
# ==========================================
class BBBEncoders:
    def __init__(self):
        self.paths = {
            "L": "/sys/bus/counter/devices/counter1/count0/count",
            "R": "/sys/bus/counter/devices/counter2/count0/count"
        }
        self.prev_counts = {"L": None, "R": None}
        
        # Initialize, configure, and enable counters
        for dev in ["counter1", "counter2"]:
            ceiling_path = f"/sys/bus/counter/devices/{dev}/count0/ceiling"
            enable_path = f"/sys/bus/counter/devices/{dev}/count0/enable"
            if os.path.exists(ceiling_path):
                try:
                    with open(ceiling_path, "w") as f:
                        f.write("4294967295")
                except Exception as e:
                    print(f"[Encoders] Warning: Could not set ceiling for {dev}: {e}")
            if os.path.exists(enable_path):
                try:
                    with open(enable_path, "w") as f:
                        f.write("1")
                except Exception as e:
                    print(f"[Encoders] Warning: Could not enable {dev}: {e}")
                    
    def read(self):
        counts = {}
        for k, path in self.paths.items():
            try:
                with open(path, "r") as f:
                    counts[k] = int(f.read().strip())
            except Exception:
                counts[k] = 0
        return counts["L"], counts["R"]

    def update_odometry(self, last_counts):
        curr_L, curr_R = self.read()
        if self.prev_counts["L"] is None:
            self.prev_counts["L"] = curr_L
            self.prev_counts["R"] = curr_R
            return 0.0, 0.0

        # Handle 32-bit rollover
        delta_L = curr_L - self.prev_counts["L"]
        delta_R = curr_R - self.prev_counts["R"]
        for d_val in ("L", "R"):
            d_var = delta_L if d_val == "L" else delta_R
            if d_var > 2**31: d_var -= 2**32
            elif d_var < -2**31: d_var += 2**32
            if d_val == "L": delta_L = d_var
            else: delta_R = d_var

        self.prev_counts["L"] = curr_L
        self.prev_counts["R"] = curr_R

        # Calculate physical distances traveled by each wheel (including multipliers and gear ratio)
        dist_L = (delta_L / CPR) * 2 * math.pi * a * LEFT_MOTOR_DIR
        dist_R = (delta_R / CPR) * 2 * math.pi * a * RIGHT_MOTOR_DIR

        return dist_L, dist_R

# ==========================================
# MPU-6050 IMU INTERFACE (I2C)
# ==========================================

# ==========================================
# PID CONTROLLER CLASS
# ==========================================
class PIDController:
    def __init__(self, Kp, Ki, Kd, max_output, min_output, max_integral=0.5):
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
            dt = 0.1
        # Proportional term
        P = self.Kp * error
        
        # Integral term with anti-windup clamping
        self.integral += error * dt
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        I = self.Ki * self.integral
        
        # Derivative term
        D = self.Kd * (error - self.prev_error) / dt
        self.prev_error = error
        
        # Total control output
        output = P + I + D
        return max(self.min_output, min(self.max_output, output))
        
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

class BBBIMU:
    def __init__(self, bus):
        # Keep signature same, but initialize BNO055
        self.bus = bus
        self.offsets = {
            "accel_offset_x": 0.0, "accel_offset_y": 0.0, "accel_offset_z": 0.0,
            "gyro_offset_x": 0.0, "gyro_offset_y": 0.0, "gyro_offset_z": 0.0
        }
        self.initialized = False
        self.sensor = None
        try:
            import board
            import adafruit_bno055
            i2c = board.I2C()
            self.sensor = adafruit_bno055.BNO055_I2C(i2c)
            # Set mode to 6-DOF IMU (magnetic immune) to avoid motor current interference
            self.sensor.mode = 0x08
            self.initialized = True
            print("[IMU] BNO055 IMU initialized successfully.")
            self.load_calibration()
        except Exception as e:
            print(f"[IMU] Warning: Failed to initialize BNO055: {e}")

    def calibrate_gyro_startup(self, samples=20, dt=0.05):
        # BNO055 has auto self-calibration, so we don't need manual calibration
        pass
                
    def load_calibration(self):
        """Restore saved BNO055 calibration offsets from JSON file if available."""
        import os, json
        cal_file = "/home/debian/Robot/bno055_calibration.json"
        if not self.initialized or self.sensor is None:
            return
        if not os.path.exists(cal_file):
            print("[IMU] No saved calibration file found.")
            return
        try:
            with open(cal_file, "r") as f:
                offsets = json.load(f)
            self.sensor.offsets_accelerometer = tuple(offsets["accel"])
            self.sensor.offsets_gyroscope     = tuple(offsets["gyro"])
            self.sensor.offsets_magnetometer  = tuple(offsets["mag"])
            if "accel_radius" in offsets:
                self.sensor.radius_accelerometer = offsets["accel_radius"]
            if "mag_radius" in offsets:
                self.sensor.radius_magnetometer  = offsets["mag_radius"]
            print(f"[IMU] Calibration offsets restored from {cal_file}")
        except Exception as e:
            print(f"[IMU] Warning: Could not restore calibration: {e}")


    def read_gyro_z(self):
        """Read gyro Z angular velocity in degrees per second."""
        if not self.initialized or self.sensor is None:
            return 0.0
        try:
            gyro = self.sensor.gyro
            if gyro is not None and gyro[2] is not None:
                gz = gyro[2] * 57.29577951308232
                # Invert gyro Z because the IMU is physically mounted upside down
                return -gz
        except Exception:
            pass
        return 0.0

    def read_absolute_yaw(self):
        """Read absolute fused yaw (heading) in radians."""
        if not self.initialized or self.sensor is None:
            return None
        try:
            euler = self.sensor.euler
            if euler is not None and euler[0] is not None:
                # euler[0] is yaw in degrees (0 to 360)
                # Convert to radians and normalize to [-pi, pi]
                yaw_rad = math.radians(euler[0])
                # BNO055 Euler yaw increases clockwise by default, so we negate it
                # to align with standard CCW coordinate frame
                yaw_ccw = -yaw_rad
                return math.atan2(math.sin(yaw_ccw), math.cos(yaw_ccw))
        except Exception:
            pass
        return None

    def read_all(self):
        """Read all accelerometer, gyroscope, and temperature readings."""
        if not self.initialized or self.sensor is None:
            return 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0
            
        try:
            accel = self.sensor.acceleration
            gyro = self.sensor.gyro
            temp = self.sensor.temperature
            
            if accel is not None and gyro is not None and temp is not None:
                if accel[0] is not None and gyro[0] is not None:
                    ax, ay, az = [val / 9.80665 for val in accel]
                    gx, gy, gz = [val * 57.29577951308232 for val in gyro]
                    # Invert gyro Z because the IMU is physically mounted upside down
                    return ax, ay, az, gx, gy, -gz, temp
            return 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0
        except Exception:
            return 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0

# ==========================================
# MAIN CLOSED-LOOP CONTROLLER
# ==========================================
def main():
    # Initialize modular UGVLogger immediately to send UDP handshake to dashboard
    logger = UGVLogger()

    # Target Pose in Inertial Frame (eta_target = [x_t; y_t; psi_t])
    # E.g. Move straight forward along X axis for 2.0 meters
    x_target = 10.0
    y_target = 0.0
    psi_target = 0.0
    
    # Initialize PID loops for linear and angular velocities
    # Kp_rho = 0.8, Ki_rho = 0.1, Kd_rho = 0.05
    # Kp_alpha = 1.5, Ki_alpha = 0.2, Kd_alpha = 0.10
    pid_v = PIDController(Kp=0.8, Ki=0.1, Kd=0.05, max_output=MAX_LINEAR_SPEED, min_output=-MAX_LINEAR_SPEED)
    pid_w = PIDController(Kp=1.5, Ki=0.2, Kd=0.10, max_output=2.0, min_output=-2.0)
    
    # Initialize hardware and encoders
    hw = BBBHardware()
    
    # Register signal handlers for clean exit on SIGTERM/SIGINT
    def sig_handler(sig, frame):
        print(f"\n[Main] Signal {sig} received. De-energizing safely...")
        hw.close()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)
    
    encoders = BBBEncoders()
    imu = BBBIMU(hw.bus)
    

    
    # Pose vector of robot in inertial frame (eta = [x; y; psi])
    x, y, psi = 0.0, 0.0, 0.0
    yaw_start = None
    rpm_L, rpm_R = 0.0, 0.0
    
    print(f"Starting straight-line control loop (dt={dt}s)...")
    print(f"Target: x={x_target:.2f}m, y={y_target:.2f}m, psi={math.degrees(psi_target):.1f}°")
    
    last_time = time.time()
    
    try:
        while True:
            loop_start = time.time()
            dt_actual = loop_start - last_time
            last_time = loop_start
            delta_L = 0
            delta_R = 0
            
            # In the first loop or if there is a timing glitch, default to nominal dt
            if dt_actual <= 0 or dt_actual > 0.5:
                dt_actual = dt
            
            # 1. Update position from encoders, yaw from IMU
            prev_L, prev_R = encoders.prev_counts["L"], encoders.prev_counts["R"]
            dist_L, dist_R = encoders.update_odometry(encoders.prev_counts)
            d_center = (dist_L + dist_R) / 2.0
            
            # Read IMU values
            ax, ay, az, gx, gy, gz, temp_c = imu.read_all()
            abs_yaw = imu.read_absolute_yaw()
            
            if abs_yaw is not None:
                if yaw_start is None:
                    yaw_start = abs_yaw
                    print(f"\n[IMU] Fused Yaw Lock: start={math.degrees(yaw_start):.1f}°")
                psi = abs_yaw - yaw_start
                # Normalize yaw to [-pi, pi]
                psi = math.atan2(math.sin(psi), math.cos(psi))
            else:
                # Fallback to gyro integration if BNO055 fails to return euler angles temporarily
                yaw_rate = math.radians(gz)
                d_theta = yaw_rate * dt_actual
                psi += d_theta
                psi = math.atan2(math.sin(psi), math.cos(psi))
            
            # Update position (Rotation matrix translation)
            x += d_center * math.cos(psi)
            y += d_center * math.sin(psi)
            
            # Calculate motor shaft RPM for telemetry
            enc_l = 0.0
            enc_r = 0.0
            if prev_L is not None and encoders.prev_counts["L"] is not None:
                delta_L = encoders.prev_counts["L"] - prev_L
                if delta_L > 2**31: delta_L -= 2**32
                elif delta_L < -2**31: delta_L += 2**32
                enc_l = (delta_L / (2400 * dt_actual)) * 60.0 * LEFT_MOTOR_DIR
                
            if prev_R is not None and encoders.prev_counts["R"] is not None:
                delta_R = encoders.prev_counts["R"] - prev_R
                if delta_R > 2**31: delta_R -= 2**32
                elif delta_R < -2**31: delta_R += 2**32
                enc_r = (delta_R / (2400 * dt_actual)) * 60.0 * RIGHT_MOTOR_DIR
                
            # --- Log Step (writes CSV and broadcasts UDP telemetry automatically) ---
            logger.log_step(
                dt_actual=dt_actual,
                x_fused=x, y_fused=y, yaw_deg=math.degrees(psi),
                cmd_l=rpm_L, cmd_r=rpm_R,
                enc_l=enc_l, enc_r=enc_r,
                raw_l=encoders.prev_counts["L"] if encoders.prev_counts["L"] is not None else 0,
                raw_r=encoders.prev_counts["R"] if encoders.prev_counts["R"] is not None else 0,
                delta_l=delta_L, delta_r=delta_R,
                ax=ax, ay=ay, az=az,
                gx=gx, gy=gy, gz=gz,
                mx=0.0, my=0.0, mz=0.0,
                x_imu=0.0, y_imu=0.0
            )
            
            # 2. Compute error to target (eta_tilda = eta_target - eta)
            dx = x_target - x
            dy = y_target - y
            
            # Distance error (rho) and angle error (alpha)
            rho = math.sqrt(dx**2 + dy**2)
            alpha = math.atan2(dy, dx) - psi
            alpha = math.atan2(math.sin(alpha), math.cos(alpha)) # Normalize to [-pi, pi]
            
            # 3. Stop check
            if rho < 0.05: # Within 5 cm
                print(f"\nTarget reached! Pose: x={x:.2f}m, y={y:.2f}m, psi={math.degrees(psi):.1f}°")
                hw.brake()
                break
                
            # 4. PID feedback tracking controller
            V = pid_v.update(rho, dt_actual)
            W = pid_w.update(alpha, dt_actual)
            
            # Clamp speeds to safe bounds
            V = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, V))
            # Safe heading angle limiting
            if V != 0:
                Wmax_safe = 2.0 * abs(V) / (2.0 * d)
                W = max(-Wmax_safe, min(Wmax_safe, W))
            
            # 5. Convert to wheel velocities (Inverse kinematics)
            VL = V - d * W
            VR = V + d * W
            
            # Convert linear speed to wheel RPMs
            w_L_des = VL / a # rad/s
            w_R_des = VR / a # rad/s
            
            rpm_L = (w_L_des / (2 * math.pi)) * 60.0
            rpm_R = (w_R_des / (2 * math.pi)) * 60.0
            
            # 6. Apply limits to motor RPMs
            peak = max(abs(rpm_L), abs(rpm_R))
            if peak > WHEEL_RPM_MAX:
                scale = WHEEL_RPM_MAX / peak
                rpm_L *= scale
                rpm_R *= scale
                
            # 7. Drive Motors (Closed-Loop Speed Control)
            hw.drive_motors(rpm_L, rpm_R, enc_l=enc_l, enc_r=enc_r, dt=dt_actual)
            
            # 8. Print diagnostics
            print(f"Pose: x={x:>5.2f} y={y:>5.2f} yaw={math.degrees(psi):>+6.1f}° | "
                  f"Cmd RPM: L={rpm_L:>+5.1f} R={rpm_R:>+5.1f} | "
                  f"Enc: L={enc_l:>+5.1f} R={enc_r:>+5.1f}", 
                  end='\r')
            
            # Maintain loop rate (10 Hz)
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
                
    except KeyboardInterrupt:
        print("\nInterrupted! Stopping robot.")
    finally:
        hw.close()
        logger.close()

if __name__ == "__main__":
    main()

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

WHEEL_RPM_MAX = 20.0  # Max physical wheel speed (RPM)
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
MAX_LINEAR_SPEED = 0.15  # Limit speed to a safe value (m/s) (reduced from 0.25 for safe ground test)

# ==========================================
# HARDWARE INTERFACES
# ==========================================
class BBBHardware:
    def __init__(self):
        self.bus = SMBus(2) if HW_AVAILABLE else None
        self.is_energized = False
        self.last_dac_L = 0
        self.last_dac_R = 0
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
        bytes_to_send = [(value >> 8) & 0x0F, value & 0xFF]
        try:
            msg = i2c_msg.write(addr, bytes_to_send)
            self.bus.i2c_rdwr(msg)
        except Exception as e:
            print(f"[HW] I2C write error on {hex(addr)}: {e}")

    def set_gpios(self, rev_L, rev_R, brake_on=False):
        if not GPIO_AVAILABLE: return
        GPIO.output(GPIO_REV_LEFT, GPIO.HIGH if rev_L else GPIO.LOW)
        GPIO.output(GPIO_REV_RIGHT, GPIO.HIGH if rev_R else GPIO.LOW)
        GPIO.output(GPIO_BRAKE_LEFT, GPIO.HIGH if brake_on else GPIO.LOW)
        GPIO.output(GPIO_BRAKE_RIGHT, GPIO.HIGH if brake_on else GPIO.LOW)

    def drive_motors(self, rpm_L, rpm_R):
        # Write direction GPIOs (LOW for forward, HIGH for reverse)
        self.set_gpios(rpm_L < 0, rpm_R < 0, brake_on=False)
        
        # Convert RPM to target DAC values
        target_dac_L = int((abs(rpm_L) / WHEEL_RPM_MAX) * DAC_MAX_VALUE)
        target_dac_R = int((abs(rpm_R) / WHEEL_RPM_MAX) * DAC_MAX_VALUE)
        
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
# MAIN CLOSED-LOOP CONTROLLER
# ==========================================
def main():
    # Target Pose in Inertial Frame (eta_target = [x_t; y_t; psi_t])
    # E.g. Move straight forward along X axis for 2.0 meters
    x_target = 2.0
    y_target = 0.0
    psi_target = 0.0
    
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
    
    # Pose vector of robot in inertial frame (eta = [x; y; psi])
    x, y, psi = 0.0, 0.0, 0.0
    
    print(f"Starting straight-line control loop (dt={dt}s)...")
    print(f"Target: x={x_target:.2f}m, y={y_target:.2f}m, psi={math.degrees(psi_target):.1f}°")
    
    try:
        while True:
            loop_start = time.time()
            
            # 1. Update pose from encoder feedback (Odometry)
            dist_L, dist_R = encoders.update_odometry(encoders.prev_counts)
            
            d_center = (dist_L + dist_R) / 2.0
            d_theta = (dist_R - dist_L) / (2.0 * d)
            
            # Integrate heading (fused rotation)
            psi += d_theta
            # Normalize yaw to [-pi, pi]
            psi = math.atan2(math.sin(psi), math.cos(psi))
            
            # Update position (Rotation matrix translation: etadot = J * zeta)
            x += d_center * math.cos(psi)
            y += d_center * math.sin(psi)
            
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
                
            # 4. Proportional feedback tracking controller
            V = Kp_rho * rho
            W = Kp_alpha * alpha
            
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
                
            # 7. Drive Motors
            hw.drive_motors(rpm_L, rpm_R)
            
            # 8. Print diagnostics
            print(f"Pose: x={x:>5.2f} y={y:>5.2f} yaw={math.degrees(psi):>+6.1f}° | "
                  f"Errors: ρ={rho:.2f}m α={math.degrees(alpha):>+6.1f}° | "
                  f"Cmd RPM: L={rpm_L:>+5.1f} R={rpm_R:>+5.1f}", 
                  end='\r')
            
            # Maintain loop rate (10 Hz)
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
                
    except KeyboardInterrupt:
        print("\nInterrupted! Stopping robot.")
    finally:
        hw.close()

if __name__ == "__main__":
    main()

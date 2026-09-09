#!/usr/bin/env python3
"""
test_forward.py
---------------
Simple diagnostic script for BeagleBone Black to run both wheels in the 
forward direction at a safe speed for 10 seconds, then stop and de-energize.

Run with sudo:
    sudo python3 test_forward.py
"""

import time
import sys
import os

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
# CONFIGURATION (Corrected from rover_control.py)
# ==========================================
# Target speed (0 to 4095)
SPEED_DAC_VALUE = 2000  # Safe moderate speed (~50% throttle)
RUN_DURATION = 10.0      # Duration in seconds

# Hardware addresses (Working UGV configuration)
DAC_LEFT_ADDR = 0x61
DAC_RIGHT_ADDR = 0x60
DAC_MAX_VALUE = 4095
DAC_ZERO_VALUE = 0

GPIO_CONTACTOR = "P8_14"
GPIO_REV_LEFT = "P8_7"
GPIO_REV_RIGHT = "P8_9"
GPIO_BRAKE_LEFT = "P8_8"
GPIO_BRAKE_RIGHT = "P8_10"

# Left and Right direction settings for forward motion:
REV_LEFT_FORWARD = False
REV_RIGHT_FORWARD = False

ENCODER_PATHS = {
    "L": "/sys/bus/counter/devices/counter1/count0/count",
    "R": "/sys/bus/counter/devices/counter2/count0/count"
}

def read_encoders():
    counts = {}
    for side in ("L", "R"):
        try:
            with open(ENCODER_PATHS[side], "r") as f:
                counts[side] = int(f.read().strip())
        except Exception:
            counts[side] = 0
    return counts["L"], counts["R"]

def write_dac(bus, address, digital_value):
    if not bus: return
    digital_value = max(0, min(digital_value, DAC_MAX_VALUE)) 
    # Use the raw 2-byte Fast Mode Write format verified in rover_control.py
    bytes_to_send = [(digital_value >> 8) & 0x0F, digital_value & 0xFF]
    try:
        msg = i2c_msg.write(address, bytes_to_send)
        bus.i2c_rdwr(msg)
    except Exception as e:
        print(f"\n[I2C ERROR] Cannot write to DAC at {hex(address)}: {e}")

def main():
    if not HW_AVAILABLE or not GPIO_AVAILABLE:
        print("Error: Missing hardware libraries (smbus2 or Adafruit_BBIO).")
        print("Please ensure this is run on the BeagleBone Black.")
        sys.exit(1)

    print("[Test] Automatically configuring pinmuxes...")
    for pin in [GPIO_REV_LEFT, GPIO_REV_RIGHT, GPIO_BRAKE_LEFT, GPIO_BRAKE_RIGHT, GPIO_CONTACTOR]:
        os.system(f"config-pin {pin} gpio >/dev/null 2>&1 || true")

    print("[Test] Initializing GPIOs...")
    for pin in [GPIO_REV_LEFT, GPIO_REV_RIGHT, GPIO_BRAKE_LEFT, GPIO_BRAKE_RIGHT, GPIO_CONTACTOR]:
        GPIO.setup(pin, GPIO.OUT)

    # Initial safe states: brakes HIGH (engaged), contactor LOW
    GPIO.output(GPIO_CONTACTOR, GPIO.LOW)
    GPIO.output(GPIO_REV_LEFT, GPIO.LOW)
    GPIO.output(GPIO_REV_RIGHT, GPIO.LOW)
    GPIO.output(GPIO_BRAKE_LEFT, GPIO.HIGH)
    GPIO.output(GPIO_BRAKE_RIGHT, GPIO.HIGH)

    # Initialize I2C Bus 2
    bus = SMBus(2)

    try:
        print("[Test] Energizing system (closing contactor)...")
        GPIO.output(GPIO_CONTACTOR, GPIO.HIGH)
        time.sleep(0.5)  # Let drivers fully boot

        print(f"[Test] Toggling direction pins (L=FORWARD, R=FORWARD)...")
        GPIO.output(GPIO_REV_LEFT, GPIO.HIGH if REV_LEFT_FORWARD else GPIO.LOW)
        GPIO.output(GPIO_REV_RIGHT, GPIO.HIGH if REV_RIGHT_FORWARD else GPIO.LOW)
        
        print("[Test] Releasing brakes...")
        GPIO.output(GPIO_BRAKE_LEFT, GPIO.LOW)
        GPIO.output(GPIO_BRAKE_RIGHT, GPIO.LOW)
        time.sleep(0.1)

        # Safely ramp up speed to prevent lockout
        print(f"[Test] Ramping speed to target {SPEED_DAC_VALUE}...")
        for speed in range(0, SPEED_DAC_VALUE + 1, 100):
            write_dac(bus, DAC_LEFT_ADDR, speed)
            write_dac(bus, DAC_RIGHT_ADDR, speed)
            time.sleep(0.05)

        print(f"[Test] Running forward for {RUN_DURATION} seconds with RPM monitoring...")
        
        # RPM measurement variables
        COUNTS_PER_REV = 2400
        GEAR_RATIO = 20.0
        
        prev_L, prev_R = read_encoders()
        prev_time = time.monotonic()
        start_time = time.monotonic()
        
        while time.monotonic() - start_time < RUN_DURATION:
            time.sleep(0.1)
            now = time.monotonic()
            dt = now - prev_time
            if dt <= 0:
                continue
            prev_time = now
            
            curr_L, curr_R = read_encoders()
            
            # Left wheel (counts down for forward)
            delta_L = curr_L - prev_L
            prev_L = curr_L
            if delta_L > 2**31: delta_L -= 2**32
            elif delta_L < -2**31: delta_L += 2**32
            rpm_L = (delta_L / (2400 * dt)) * 60.0 * -1
            
            # Right wheel (counts up for forward)
            delta_R = curr_R - prev_R
            prev_R = curr_R
            if delta_R > 2**31: delta_R -= 2**32
            elif delta_R < -2**31: delta_R += 2**32
            rpm_R = (delta_R / (2400 * dt)) * 60.0 * 1
            
            print(f"Time: {now - start_time:.1f}s | Wheel RPM -> L: {rpm_L:>+7.2f} | R: {rpm_R:>+7.2f}", end="\r")
        print()

        print("[Test] Time elapsed. Stopping motors...")
        write_dac(bus, DAC_LEFT_ADDR, DAC_ZERO_VALUE)
        write_dac(bus, DAC_RIGHT_ADDR, DAC_ZERO_VALUE)

    except KeyboardInterrupt:
        print("\n[Test] Interrupted! Stopping immediately.")
    finally:
        print("[Test] De-energizing contactor and cleaning up...")
        # Open contactor (cut power)
        GPIO.output(GPIO_CONTACTOR, GPIO.LOW)
        
        # Turn off direction & enable brakes for safety
        GPIO.output(GPIO_REV_LEFT, GPIO.LOW)
        GPIO.output(GPIO_REV_RIGHT, GPIO.LOW)
        GPIO.output(GPIO_BRAKE_LEFT, GPIO.HIGH)
        GPIO.output(GPIO_BRAKE_RIGHT, GPIO.HIGH)
        
        # Close I2C bus and clear GPIO configuration
        bus.close()
        GPIO.cleanup()
        print("[Test] Cleanup complete. Safe to exit.")

if __name__ == "__main__":
    main()

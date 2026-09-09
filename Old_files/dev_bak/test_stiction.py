#!/usr/bin/env python3
import time
import sys
import os
import signal

try:
    from smbus2 import SMBus, i2c_msg
    import Adafruit_BBIO.GPIO as GPIO
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False

# Configuration
DAC_LEFT_ADDR = 0x61
DAC_RIGHT_ADDR = 0x60
DAC_MAX_VALUE = 4095
DAC_ZERO_VALUE = 0

GPIO_CONTACTOR = "P8_14"
GPIO_REV_LEFT = "P8_7"
GPIO_REV_RIGHT = "P8_9"
GPIO_BRAKE_LEFT = "P8_8"
GPIO_BRAKE_RIGHT = "P8_10"

ENCODER_PATHS = {
    "L": "/sys/bus/counter/devices/counter1/count0/count",
    "R": "/sys/bus/counter/devices/counter2/count0/count"
}

class TestHardware:
    def __init__(self):
        self.bus = SMBus(2) if HW_AVAILABLE else None
        if HW_AVAILABLE:
            for pin in [GPIO_REV_LEFT, GPIO_REV_RIGHT, GPIO_BRAKE_LEFT, GPIO_BRAKE_RIGHT, GPIO_CONTACTOR]:
                os.system(f"config-pin {pin} gpio >/dev/null 2>&1 || true")
                GPIO.setup(pin, GPIO.OUT)
            GPIO.output(GPIO_CONTACTOR, GPIO.LOW)
            GPIO.output(GPIO_REV_LEFT, GPIO.LOW)
            GPIO.output(GPIO_REV_RIGHT, GPIO.LOW)
            GPIO.output(GPIO_BRAKE_LEFT, GPIO.HIGH)
            GPIO.output(GPIO_BRAKE_RIGHT, GPIO.HIGH)

    def energize(self):
        print("[HW] Energizing contactor...")
        GPIO.output(GPIO_CONTACTOR, GPIO.HIGH)
        time.sleep(0.5)
        print("[HW] Releasing brakes...")
        GPIO.output(GPIO_BRAKE_LEFT, GPIO.LOW)
        GPIO.output(GPIO_BRAKE_RIGHT, GPIO.LOW)
        time.sleep(0.1)

    def write_dacs(self, val_L, val_R):
        self._write_single(DAC_LEFT_ADDR, val_L)
        self._write_single(DAC_RIGHT_ADDR, val_R)

    def _write_single(self, addr, val):
        if not self.bus: return
        val = max(0, min(val, DAC_MAX_VALUE))
        bytes_to_send = [(val >> 8) & 0x0F, val & 0xFF]
        try:
            msg = i2c_msg.write(addr, bytes_to_send)
            self.bus.i2c_rdwr(msg)
        except Exception as e:
            print(f"I2C write error to {hex(addr)}: {e}")

    def close(self):
        print("[HW] De-energizing safely...")
        if self.bus:
            self.write_dacs(0, 0)
        GPIO.output(GPIO_BRAKE_LEFT, GPIO.HIGH)
        GPIO.output(GPIO_BRAKE_RIGHT, GPIO.HIGH)
        GPIO.output(GPIO_CONTACTOR, GPIO.LOW)
        if self.bus: self.bus.close()
        GPIO.cleanup()

def read_encoders():
    counts = {}
    for side in ("L", "R"):
        try:
            with open(ENCODER_PATHS[side], "r") as f:
                counts[side] = int(f.read().strip())
        except Exception:
            counts[side] = 0
    return counts["L"], counts["R"]

def main():
    hw = TestHardware()
    def sig_handler(sig, frame):
        hw.close()
        sys.exit(0)
    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    hw.energize()

    # Step DAC throttle test points
    test_dacs = [400, 600, 800, 1000, 1200, 1500, 1800, 2000]
    
    print("\nStarting stiction threshold mapping test. The UGV is jacked up.")
    print(f"{'DAC Command':^12} | {'Left Wheel RPM':^16} | {'Right Wheel RPM':^16}")
    print("-" * 52)

    try:
        for dac_val in test_dacs:
            # Command DAC
            hw.write_dacs(dac_val, dac_val)
            
            # Let speed stabilize for 2 seconds
            time.sleep(2.0)
            
            # Measure over 2.0 seconds
            prev_L, prev_R = read_encoders()
            t0 = time.monotonic()
            
            time.sleep(2.0)
            
            curr_L, curr_R = read_encoders()
            t1 = time.monotonic()
            dt = t1 - t0
            
            # Calculate RPM
            delta_L = curr_L - prev_L
            delta_R = curr_R - prev_R
            
            if delta_L > 2**31: delta_L -= 2**32
            elif delta_L < -2**31: delta_L += 2**32
            
            if delta_R > 2**31: delta_R -= 2**32
            elif delta_R < -2**31: delta_R += 2**32
            
            # Divide by CPR (2400) and gear ratio (20) to get wheel RPM
            # Left wheel counts down (negative delta) for forward
            rpm_L = (delta_L / (2400 * dt)) * 60.0 * -1 / 20.0
            # Right wheel counts up (positive delta) for forward
            rpm_R = (delta_R / (2400 * dt)) * 60.0 * 1 / 20.0
            
            print(f"{dac_val:^12d} | {rpm_L:>+15.2f} | {rpm_R:>+15.2f}")
            
    finally:
        hw.close()

if __name__ == '__main__':
    main()

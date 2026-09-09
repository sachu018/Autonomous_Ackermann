#!/usr/bin/env python3
"""
test_full_reverse.py (Diagnostic Sweep with Encoder Feedback)
-------------------------------------------------------------
Tests both BLDC motors in reverse by performing a forward-to-reverse boot transition.
Ramps up the DAC throttle voltage and reads live encoder RPMs to observe the exact
stiction point and speed plateau.
"""

import time
import os
import sys
import signal

try:
    import smbus2
    import Adafruit_BBIO.GPIO as GPIO
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False

# GPIO Pins
GPIO_CONTACTOR = "P8_14"
GPIO_BRAKE_LEFT = "P8_8"
GPIO_BRAKE_RIGHT = "P8_10"
GPIO_REV_LEFT = "P8_7"
GPIO_REV_RIGHT = "P8_9"

# DAC Addresses
DAC_LEFT_ADDR = 0x61
DAC_RIGHT_ADDR = 0x60
DAC_MAX_VALUE = 4095
DAC_ZERO_VALUE = 0

# Encoder Settings
ENCODER_PATHS = {
    "L": "/sys/bus/counter/devices/counter1/count0/count",  # eQEP1 → P8_35/33
    "R": "/sys/bus/counter/devices/counter2/count0/count",  # eQEP2 → P8_12/11
}
COUNTS_PER_REV = 2400

# Encoder tracking state
_enc_prev = {"L": None, "R": None}
_enc_time = time.monotonic()
_enc_rpm = {"L": 0.0, "R": 0.0}
_enc_ok = {k: os.path.exists(p) for k, p in ENCODER_PATHS.items()}

def _read_enc_count(key):
    try:
        with open(ENCODER_PATHS[key], 'r') as f:
            return int(f.read().strip())
    except Exception:
        return None

def _update_encoders():
    """Read both encoder counters and update _enc_rpm dict."""
    global _enc_time
    now = time.monotonic()
    dt = now - _enc_time
    _enc_time = now
    if dt <= 0:
        return
    for key in ("L", "R"):
        if not _enc_ok[key]:
            continue
        curr = _read_enc_count(key)
        if curr is None:
            _enc_rpm[key] = 0.0
            _enc_prev[key] = None
            continue
        if _enc_prev[key] is None:
            _enc_prev[key] = curr
            continue
        delta = curr - _enc_prev[key]
        _enc_prev[key] = curr
        # Handle 32-bit rollover
        if delta > 2**31: delta -= 2**32
        if delta < -2**31: delta += 2**32
        # RPM = (delta / counts_per_rev) / dt * 60
        _enc_rpm[key] = (delta / (COUNTS_PER_REV * dt)) * 60.0

def write_dac(bus, addr, value):
    upper = (value >> 4) & 0xFF
    lower = (value << 4) & 0xFF
    try:
        bus.write_i2c_block_data(addr, 0x40, [upper, lower])
    except Exception as e:
        print(f"\n[ERROR] I2C write failure: {e}")

def main():
    if not HW_AVAILABLE:
        print("[ERROR] Missing hardware libraries. Run on BeagleBone.")
        return

    print("[1/6] Configuring BeagleBone Black pinmuxes...")
    for pin in [GPIO_REV_LEFT, GPIO_REV_RIGHT, GPIO_BRAKE_LEFT, GPIO_BRAKE_RIGHT, GPIO_CONTACTOR]:
        os.system(f"config-pin {pin} gpio >/dev/null 2>&1 || true")
        
    GPIO.setup(GPIO_CONTACTOR, GPIO.OUT)
    GPIO.setup(GPIO_BRAKE_LEFT, GPIO.OUT)
    GPIO.setup(GPIO_BRAKE_RIGHT, GPIO.OUT)
    GPIO.setup(GPIO_REV_LEFT, GPIO.OUT)
    GPIO.setup(GPIO_REV_RIGHT, GPIO.OUT)

    # Initial state
    GPIO.output(GPIO_REV_LEFT, GPIO.LOW)
    GPIO.output(GPIO_REV_RIGHT, GPIO.LOW)
    GPIO.output(GPIO_BRAKE_LEFT, GPIO.HIGH)
    GPIO.output(GPIO_BRAKE_RIGHT, GPIO.HIGH)
    GPIO.output(GPIO_CONTACTOR, GPIO.LOW)
    
    bus = smbus2.SMBus(2)

    # Check encoder status
    for k, ok in _enc_ok.items():
        if not ok:
            print(f"[WARNING] Encoder {k} path not found. RPM will show 0.")

    def safe_cleanup(sig, frame):
        print("\n\n[SHUTDOWN] Emergency stop! Safe-state active...")
        write_dac(bus, DAC_LEFT_ADDR, 0)
        write_dac(bus, DAC_RIGHT_ADDR, 0)
        GPIO.output(GPIO_BRAKE_LEFT, GPIO.HIGH)
        GPIO.output(GPIO_BRAKE_RIGHT, GPIO.HIGH)
        GPIO.output(GPIO_REV_LEFT, GPIO.LOW)
        GPIO.output(GPIO_REV_RIGHT, GPIO.LOW)
        GPIO.output(GPIO_CONTACTOR, GPIO.LOW)
        bus.close()
        GPIO.cleanup()
        print("[SHUTDOWN] Done.")
        sys.exit(0)

    signal.signal(signal.SIGINT, safe_cleanup)
    signal.signal(signal.SIGTERM, safe_cleanup)

    print("\n=======================================================")
    print("WARNING: ROBOT WILL RUN FORWARD (2s) THEN REVERSE RAMP.")
    print("Live encoder RPM values will be displayed.")
    print("Press Ctrl+C to emergency stop at any time.")
    print("=======================================================\n")
    
    input("Press Enter to start the test...")

    try:
        # Step 1: Power up in Forward
        print("[2/6] Energizing main contactor relay...")
        GPIO.output(GPIO_CONTACTOR, GPIO.HIGH)
        time.sleep(0.5)

        # Step 2: Release Brakes
        print("[3/6] Releasing brakes in FORWARD mode...")
        GPIO.output(GPIO_REV_LEFT, GPIO.LOW)
        GPIO.output(GPIO_REV_RIGHT, GPIO.LOW)
        GPIO.output(GPIO_BRAKE_LEFT, GPIO.LOW)
        GPIO.output(GPIO_BRAKE_RIGHT, GPIO.LOW)
        time.sleep(0.1)

        # Initialize encoder time baseline
        global _enc_time
        _enc_time = time.monotonic()

        # Step 3: Run Forward Confirmation
        print("[4/6] Running FORWARD at 2000 DAC for 2 seconds...")
        write_dac(bus, DAC_LEFT_ADDR, 2000)
        write_dac(bus, DAC_RIGHT_ADDR, 2000)
        
        # Poll encoders during forward run
        for _ in range(20):
            _update_encoders()
            print(f"  Forwarding | RPM L: {_enc_rpm['L']:>+7.1f} R: {_enc_rpm['R']:>+7.1f}", end='\r')
            sys.stdout.flush()
            time.sleep(0.1)
        print()

        # Step 4: Stop Throttle
        print("[5/6] Stopping throttle (writing 0 DAC) and preparing reverse transition...")
        write_dac(bus, DAC_LEFT_ADDR, 0)
        write_dac(bus, DAC_RIGHT_ADDR, 0)
        time.sleep(0.5)

        # Step 5: Shift to Reverse
        print("[6/6] Toggling direction to REVERSE (REV = HIGH)...")
        GPIO.output(GPIO_REV_LEFT, GPIO.HIGH)
        GPIO.output(GPIO_REV_RIGHT, GPIO.HIGH)
        time.sleep(0.15) 

        # Step 6: Ramp Sweep in Reverse with Encoder Telemetry
        print("\nStarting REVERSE throttle sweep. Note when RPM changes and when it plateaus:\n")
        
        start_dac = 1100
        end_dac = 4095
        step = 100

        for dac_val in range(start_dac, end_dac + 1, step):
            write_dac(bus, DAC_LEFT_ADDR, dac_val)
            write_dac(bus, DAC_RIGHT_ADDR, dac_val)
            
            # Poll encoders multiple times during this step's 0.5s duration
            for _ in range(5):
                _update_encoders()
                print(f"  DAC: {dac_val:<4} (~{dac_val/4095 * 5.0:.2f}V) | RPM L: {_enc_rpm['L']:>+7.1f} R: {_enc_rpm['R']:>+7.1f}", end='\r')
                sys.stdout.flush()
                time.sleep(0.1)

        print("\n\nReached max reverse. Holding for 5 seconds...")
        for _ in range(50):
            _update_encoders()
            print(f"  Holding | RPM L: {_enc_rpm['L']:>+7.1f} R: {_enc_rpm['R']:>+7.1f}", end='\r')
            sys.stdout.flush()
            time.sleep(0.1)
        print()

    except Exception as e:
        print(f"\n[EXCEPTION] Run-time error: {e}")
    finally:
        safe_cleanup(None, None)

if __name__ == '__main__':
    main()

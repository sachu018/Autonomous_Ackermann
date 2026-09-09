#!/usr/bin/env python3

"""
BNO055 Auto-Calibration Script for UGV
---------------------------------------
Phases:
  1. STILL   : Stop all motors for 5 seconds  → Gyroscope  calibrates to Lvl 3
  2. SPIN    : Slow spot-turn in place         → Magnetometer calibrates to Lvl 3
  3. SAVE    : Write calibration offsets to JSON file

Run this script with: sudo python3 auto_calibrate_bno055.py
"""

import time
import json
import sys
import os
import smbus2
import board
import adafruit_bno055
from adafruit_mcp4725 import MCP4725

# ── Hardware Addresses ────────────────────────────────────────────────────────
DAC_LEFT_ADDR  = 0x61
DAC_RIGHT_ADDR = 0x60
I2C_BUS_NUMBER = 2

# GPIO pins (BCM names via Adafruit Blinka for BeagleBone)
import Adafruit_BBIO.GPIO as GPIO
GPIO_REV_LEFT    = "P8_7"
GPIO_REV_RIGHT   = "P8_9"
GPIO_BRAKE_LEFT  = "P8_8"
GPIO_BRAKE_RIGHT = "P8_10"
GPIO_CONTACTOR   = "P8_14"

# ── Calibration output file ───────────────────────────────────────────────────
CAL_FILE = "/home/debian/Robot/bno055_calibration.json"

# ── Motor DAC values ──────────────────────────────────────────────────────────
DAC_ZERO  = 0
# Slow spin speed: ~30% of max DAC
SPIN_DAC  = 1200   # ~0.7 RPM wheel speed

def setup_gpio():
    GPIO.setup(GPIO_CONTACTOR,   GPIO.OUT)
    GPIO.setup(GPIO_BRAKE_LEFT,  GPIO.OUT)
    GPIO.setup(GPIO_BRAKE_RIGHT, GPIO.OUT)
    GPIO.setup(GPIO_REV_LEFT,    GPIO.OUT)
    GPIO.setup(GPIO_REV_RIGHT,   GPIO.OUT)

def stop_motors(bus, dac_l, dac_r):
    """Stop both motors and apply brakes."""
    dac_l.raw_value = DAC_ZERO
    dac_r.raw_value = DAC_ZERO
    GPIO.output(GPIO_BRAKE_LEFT,  GPIO.HIGH)
    GPIO.output(GPIO_BRAKE_RIGHT, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(GPIO_BRAKE_LEFT,  GPIO.LOW)
    GPIO.output(GPIO_BRAKE_RIGHT, GPIO.LOW)

def set_forward(dac_l, dac_r, speed):
    """Both wheels forward."""
    GPIO.output(GPIO_REV_LEFT,   GPIO.LOW)
    GPIO.output(GPIO_REV_RIGHT,  GPIO.LOW)
    dac_l.raw_value = speed
    dac_r.raw_value = speed

def set_spot_turn(dac_l, dac_r, speed, clockwise=True):
    """Spot turn: left wheel forward, right wheel reverse."""
    if clockwise:
        GPIO.output(GPIO_REV_LEFT,  GPIO.LOW)   # left forward
        GPIO.output(GPIO_REV_RIGHT, GPIO.HIGH)  # right reverse
    else:
        GPIO.output(GPIO_REV_LEFT,  GPIO.HIGH)  # left reverse
        GPIO.output(GPIO_REV_RIGHT, GPIO.LOW)   # right forward
    dac_l.raw_value = speed
    dac_r.raw_value = speed

def get_cal_status(sensor):
    """Return (sys, gyro, accel, mag) calibration status."""
    try:
        cal = sensor.calibration_status
        if cal is None:
            return (0, 0, 0, 0)
        return cal
    except Exception:
        return (0, 0, 0, 0)

def save_calibration(sensor):
    """Read BNO055 offsets and save to JSON file."""
    try:
        offsets = {
            "accel":    list(sensor.offsets_accelerometer),
            "gyro":     list(sensor.offsets_gyroscope),
            "mag":      list(sensor.offsets_magnetometer),
            "accel_radius": sensor.radius_accelerometer,
            "mag_radius":   sensor.radius_magnetometer,
        }
        with open(CAL_FILE, "w") as f:
            json.dump(offsets, f, indent=2)
        print("\n[Calibration] Offsets saved to {}".format(CAL_FILE))
        print(json.dumps(offsets, indent=2))
        return True
    except Exception as e:
        print("\n[Calibration] Error saving offsets: {}".format(e))
        return False

def run_calibration():
    print("=" * 60)
    print("  BNO055 Auto-Calibration Routine")
    print("=" * 60)

    # ── Initialize GPIO first ─────────────────────────────────────────────────
    print("[Init] Setting up GPIO...")
    try:
        setup_gpio()
    except Exception as e:
        print("ERROR: Failed to set up GPIO: {}".format(e))
        sys.exit(1)

    # ── Engage contactor to power up the DACs ─────────────────────────────────
    print("[Init] Engaging contactor to power motor controllers...")
    try:
        GPIO.output(GPIO_CONTACTOR, GPIO.HIGH)
        time.sleep(1.0) # Wait for power rails to stabilize
        print("[Init] Contactor engaged.")
    except Exception as e:
        print("ERROR: Failed to engage contactor: {}".format(e))
        sys.exit(1)

    # ── Initialize BNO055 ─────────────────────────────────────────────────────
    print("\n[Init] Connecting to BNO055...")
    try:
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c)
        print("[Init] BNO055 connected.")
    except Exception as e:
        print("ERROR: Could not connect to BNO055: {}".format(e))
        GPIO.output(GPIO_CONTACTOR, GPIO.LOW)
        sys.exit(1)

    # ── Initialize DAC (I2C Motor Controllers) ────────────────────────────────
    print("[Init] Connecting to motor DACs...")
    try:
        bus = smbus2.SMBus(I2C_BUS_NUMBER)
        dac_l = MCP4725(board.I2C(), address=DAC_LEFT_ADDR)
        dac_r = MCP4725(board.I2C(), address=DAC_RIGHT_ADDR)
        print("[Init] DACs connected.")
    except Exception as e:
        print("ERROR: Could not connect to DACs: {}".format(e))
        GPIO.output(GPIO_CONTACTOR, GPIO.LOW)
        sys.exit(1)

    try:
        # ── PHASE 1: STILL ────────────────────────────────────────────────────
        print("\n[Phase 1] STILL — Gyroscope calibration")
        print("  Stopping all motors for 5 seconds...")
        stop_motors(None, dac_l, dac_r)

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            c_sys, c_gyro, c_acc, c_mag = get_cal_status(sensor)
            print("\r  Sys:{}  Gyro:{}  Accel:{}  Mag:{}  ".format(c_sys, c_gyro, c_acc, c_mag), end="", flush=True)
            if c_gyro >= 3:
                print("\n  [Phase 1] Gyroscope calibrated! (Lvl {})".format(c_gyro))
                break
            time.sleep(0.5)
        else:
            print("\n  [Phase 1] Gyroscope not fully calibrated yet, proceeding anyway...")

        # ── PHASE 2: SLOW SPIN ────────────────────────────────────────────────
        print("\n[Phase 2] SPIN — Magnetometer calibration (slow spot-turn)")
        print("  Robot will spin slowly. This may take up to 60 seconds...")
        print("  Watch Mag level rise from 0 -> 1 -> 2 -> 3\n")

        deadline = time.monotonic() + 90.0
        clockwise = True
        spin_start = time.monotonic()

        while time.monotonic() < deadline:
            c_sys, c_gyro, c_acc, c_mag = get_cal_status(sensor)
            elapsed = time.monotonic() - spin_start
            print("\r  [{:5.1f}s]  Sys:{}  Gyro:{}  Accel:{}  Mag:{}  ".format(elapsed, c_sys, c_gyro, c_acc, c_mag), end="", flush=True)

            if c_mag >= 3:
                print("\n  [Phase 2] Magnetometer calibrated! (Lvl {})".format(c_mag))
                break

            # Alternate spin direction every 20 seconds
            if int(elapsed) % 20 == 0 and elapsed > 1:
                clockwise = not clockwise

            set_spot_turn(dac_l, dac_r, SPIN_DAC, clockwise=clockwise)
            time.sleep(0.3)
        else:
            print("\n  [Phase 2] WARNING: Magnetometer did not reach Lvl 3 within 90 seconds.")
            print("  Saving whatever calibration data we have...")

        # ── STOP MOTORS ───────────────────────────────────────────────────────
        stop_motors(None, dac_l, dac_r)
        print("\n[Stop] Motors stopped.")

        # ── PHASE 3: FINAL STATUS & SAVE ─────────────────────────────────────
        time.sleep(0.5)
        c_sys, c_gyro, c_acc, c_mag = get_cal_status(sensor)
        print("\n[Phase 3] Final calibration status:")
        print("  System:        Lvl {}".format(c_sys))
        print("  Gyroscope:     Lvl {}".format(c_gyro))
        print("  Accelerometer: Lvl {}".format(c_acc))
        print("  Magnetometer:  Lvl {}".format(c_mag))

        success = save_calibration(sensor)
        if success:
            print("\n[Done] Calibration complete! Robot will now use these offsets on next boot.")
        else:
            print("\n[Done] Calibration routine complete (but offsets could not be saved).")

    except KeyboardInterrupt:
        print("\n[Interrupted] Stopping motors...")
        stop_motors(None, dac_l, dac_r)
    finally:
        stop_motors(None, dac_l, dac_r)
        GPIO.output(GPIO_CONTACTOR, GPIO.LOW)
        GPIO.cleanup()
        print("[Cleanup] GPIO cleaned up. Done.")

if __name__ == "__main__":
    run_calibration()

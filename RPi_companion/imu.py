#!/usr/bin/env python3
# imu.py — DFRobot Fermion BNO055 9-axis IMU driver (Raspberry Pi 5, I2C1)
#
# Wiring (RPi 40-pin header, I2C bus 1):
#   BNO055 VCC -> Pin 1  (3V3)
#   BNO055 SDA -> Pin 3  (GPIO2 / SDA)
#   BNO055 SCL -> Pin 5  (GPIO3 / SCL)
#   BNO055 GND -> Pin 6  (GND)
#
# Default I2C address 0x28 (ADR pin floating/low on the Fermion board). If
# i2cdetect -y 1 shows the device at 0x29 instead, pass address=0x29 to IMU().
#
# Fault-handling philosophy matches the STM32 firmware's ADS1115 driver
# (Rover_closed_loop/Core/Src/ads1115.c): a transient I2C glitch holds the
# last valid reading rather than immediately reporting invalid, and only
# after IMU_FAULT_THRESHOLD consecutive failures is the reading actually
# flagged invalid. Kept consistent deliberately — see architecture.md §3.

import time

try:
    import board
    import adafruit_bno055
    _HW_AVAILABLE = True
    _STUB_REASON = None
except ImportError:
    # The package genuinely isn't installed in whatever Python is running
    # this — almost always means the rover-venv wasn't activated (system
    # python3 doesn't have these; PEP 668 blocks installing them there).
    _HW_AVAILABLE = False
    _STUB_REASON = (
        "adafruit-blinka/adafruit-circuitpython-bno055 not importable in "
        "this Python. Did you forget: source ~/rover-venv/bin/activate ? "
        "(sudo also won't see the venv — don't use sudo for this script.)"
    )
except NotImplementedError:
    # This is what Blinka itself raises when it can't identify the board at
    # all — i.e. this code really is running off the actual Raspberry Pi
    # (e.g. imported on a dev laptop instead).
    _HW_AVAILABLE = False
    _STUB_REASON = "Blinka could not identify this board — not running on the actual Raspberry Pi?"

IMU_FAULT_THRESHOLD = 5          # consecutive bad reads before valid=False
DEFAULT_I2C_ADDRESS  = 0x28

# Fill this in from calibrate_imu.py's printed output after running it on
# THIS sensor in THIS mounting — set to None to skip preload and calibrate
# fresh every boot (slower to lock, but always correct if the mounting is
# still being finalized). Example, once you have real numbers:
#   IMU_CALIBRATION_OFFSETS = {
#       "accel_offset": (x, y, z), "mag_offset": (x, y, z),
#       "gyro_offset": (x, y, z), "accel_radius": r_a, "mag_radius": r_m,
#   }
IMU_CALIBRATION_OFFSETS = None


class IMUData:
    """One IMU sample. Angles in degrees, rates in rad/s, matching the units
    already used elsewhere in this project (see ackermann_config.h angles in
    degrees, localization.py yaw in radians only internally)."""
    __slots__ = (
        "heading_deg",   # Compass heading, 0-360 clockwise from magnetic North
        "roll_deg",
        "pitch_deg",
        "gyro_z_rads",   # Yaw rate, rad/s (matches ismc_controller.py's cmd_w units)
        "accel_x_ms2",
        "accel_y_ms2",
        "calib_sys",     # 0-3 each, per BNO055 calibration_status
        "calib_gyro",
        "calib_accel",
        "calib_mag",
        "valid",
    )

    def __init__(self):
        self.heading_deg = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.gyro_z_rads = 0.0
        self.accel_x_ms2 = 0.0
        self.accel_y_ms2 = 0.0
        self.calib_sys = 0
        self.calib_gyro = 0
        self.calib_accel = 0
        self.calib_mag = 0
        self.valid = False

    def is_fully_calibrated(self) -> bool:
        """Gyro + Accel are what matters for a stable heading estimate; Mag
        drifts with local field distortion and Sys often lags behind the
        other three even when they're solid — see calibrate_imu.py."""
        return self.calib_gyro == 3 and self.calib_accel == 3


class IMU:
    def __init__(self, address: int = DEFAULT_I2C_ADDRESS):
        self._sensor = None
        self._fault_count = 0
        self._last = IMUData()  # all-zero, valid=False until first good read

        if not _HW_AVAILABLE:
            print(f"[IMU] Running in stub mode (no hardware). {_STUB_REASON}")
            return

        try:
            i2c = board.I2C()  # uses the default bus (GPIO2/GPIO3, bus 1)
            self._sensor = adafruit_bno055.BNO055_I2C(i2c, address=address)
            print(f"[IMU] BNO055 initialized at 0x{address:02X}.")
        except Exception as e:
            print(f"[IMU] Failed to initialize BNO055: {e}")
            self._sensor = None
            return

        if IMU_CALIBRATION_OFFSETS is not None:
            try:
                o = IMU_CALIBRATION_OFFSETS
                self._sensor.offsets_accelerometer = o["accel_offset"]
                self._sensor.offsets_magnetometer  = o["mag_offset"]
                self._sensor.offsets_gyroscope     = o["gyro_offset"]
                self._sensor.radius_accelerometer  = o["accel_radius"]
                self._sensor.radius_magnetometer   = o["mag_radius"]
                print("[IMU] Preloaded saved calibration offsets.")
            except Exception as e:
                # Non-fatal — sensor still works, just needs a fresh
                # calibration ritual (still-still/6-orientation/figure-8).
                print(f"[IMU] Warning: failed to preload calibration offsets: {e}")

    def is_healthy(self) -> bool:
        return self._sensor is not None and self._fault_count < IMU_FAULT_THRESHOLD

    def read(self) -> IMUData:
        """Read one sample. Call at your guidance loop's rate. Holds the last
        valid sample across transient I2C glitches; only flips valid=False
        after IMU_FAULT_THRESHOLD consecutive failures."""
        if self._sensor is None:
            return self._last

        try:
            euler = self._sensor.euler            # (heading, roll, pitch) deg, or (None,None,None) pre-fusion-lock
            gyro = self._sensor.gyro               # (x, y, z) rad/s
            accel = self._sensor.acceleration      # (x, y, z) m/s^2, gravity-inclusive
            cal = self._sensor.calibration_status  # (sys, gyro, accel, mag), 0-3 each

            if euler[0] is None or gyro is None or accel is None or cal is None:
                raise ValueError("sensor returned an incomplete/None sample "
                                  "(fusion not yet locked)")

            self._last.heading_deg = float(euler[0])
            self._last.roll_deg    = float(euler[1])
            self._last.pitch_deg   = float(euler[2])
            self._last.gyro_z_rads = float(gyro[2])
            self._last.accel_x_ms2 = float(accel[0])
            self._last.accel_y_ms2 = float(accel[1])
            self._last.calib_sys   = int(cal[0])
            self._last.calib_gyro  = int(cal[1])
            self._last.calib_accel = int(cal[2])
            self._last.calib_mag   = int(cal[3])
            self._last.valid       = True
            self._fault_count      = 0

        except Exception as e:
            self._fault_count += 1
            if self._fault_count >= IMU_FAULT_THRESHOLD:
                self._last.valid = False
            # else: hold the previous sample's values, just don't refresh them

        return self._last


if __name__ == "__main__":
    # Quick standalone smoke test — prefer test_imu.py for a live readout.
    imu = IMU()
    for _ in range(5):
        d = imu.read()
        print(f"heading={d.heading_deg:.1f} roll={d.roll_deg:.1f} "
              f"pitch={d.pitch_deg:.1f} valid={d.valid} "
              f"cal(sys/gyro/accel/mag)={d.calib_sys}/{d.calib_gyro}/{d.calib_accel}/{d.calib_mag}")
        time.sleep(0.5)

# hardware.py
# Low-level hardware interface for BeagleBone Black GPIO and I2C SMBus.

import time
import os

try:
    import smbus2
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False

try:
    import Adafruit_BBIO.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

from config import (
    GPIO_CONTACTOR, GPIO_REV_LEFT, GPIO_REV_RIGHT,
    GPIO_BRAKE_LEFT, GPIO_BRAKE_RIGHT
)

class BBBHardware:
    def __init__(self):
        self.bus = None
        self.closed = False
        
        # Configure I2C and UART pinmuxes programmatically
        os.system("config-pin P9_19 i2c >/dev/null 2>&1 || true")
        os.system("config-pin P9_20 i2c >/dev/null 2>&1 || true")
        os.system("config-pin P8_37 uart >/dev/null 2>&1 || true")
        os.system("config-pin P8_38 uart >/dev/null 2>&1 || true")
        
        # Initialize I2C Bus 2
        if HW_AVAILABLE:
            try:
                self.bus = smbus2.SMBus(2)
                print("[Hardware] I2C Bus 2 opened successfully.")
            except Exception as e:
                print(f"[Hardware] Warning: Failed to open I2C Bus 2: {e}")
        else:
            print("[Hardware] SMBus library missing. Running in simulated I2C mode.")

        # Initialize GPIO Pins
        if GPIO_AVAILABLE:
            try:
                # Ensure pins are configured as GPIO outputs in sysfs
                for pin in [GPIO_CONTACTOR, GPIO_REV_LEFT, GPIO_REV_RIGHT, GPIO_BRAKE_LEFT, GPIO_BRAKE_RIGHT]:
                    os.system(f"config-pin {pin} gpio >/dev/null 2>&1 || true")
                    GPIO.setup(pin, GPIO.OUT)
                    GPIO.output(pin, GPIO.LOW)
                
                # Active-low brakes are initially HIGH (meaning brakes are engaged at startup)
                GPIO.output(GPIO_BRAKE_LEFT, GPIO.HIGH)
                GPIO.output(GPIO_BRAKE_RIGHT, GPIO.HIGH)
                print("[Hardware] GPIO pins configured. Brakes engaged, contactor open.")
            except Exception as e:
                print(f"[Hardware] Warning: GPIO setup failed: {e}")
        else:
            print("[Hardware] Adafruit_BBIO library missing. Running in simulated GPIO mode.")

    def set_gpio(self, pin, state: bool):
        """Write digital output state to a GPIO pin."""
        if GPIO_AVAILABLE:
            try:
                GPIO.output(pin, GPIO.HIGH if state else GPIO.LOW)
            except Exception as e:
                print(f"[Hardware] GPIO write error on {pin}: {e}")
        else:
            pass

    def write_dac(self, addr: int, value: int):
        """Write a 12-bit analog value to the specified MCP4725 DAC I2C address."""
        if self.bus is not None:
            # Clamp 12-bit value
            value = max(0, min(value, 4095))
            # Format bytes for MCP4725 fast write protocol (no EEPROM write)
            upper = (value >> 4) & 0xFF
            lower = (value << 4) & 0xFF
            try:
                # Write command 0x40 to write to DAC register
                self.bus.write_i2c_block_data(addr, 0x40, [upper, lower])
            except Exception as e:
                pass  # Suppress transient bus read/write errors to prevent control loops from blocking
        else:
            pass

    def close(self):
        """Release hardware resources safely."""
        if self.closed:
            return
        self.closed = True
        
        # Stop SMBus I2C
        if self.bus is not None:
            try:
                self.bus.close()
                print("[Hardware] I2C Bus closed.")
            except Exception:
                pass
                
        # Cleanup GPIOs
        if GPIO_AVAILABLE:
            try:
                # Force pins LOW
                for pin in [GPIO_CONTACTOR, GPIO_REV_LEFT, GPIO_REV_RIGHT, GPIO_BRAKE_LEFT, GPIO_BRAKE_RIGHT]:
                    GPIO.output(pin, GPIO.LOW)
                GPIO.cleanup()
                print("[Hardware] GPIO cleanup completed.")
            except Exception:
                pass


class BBBEncoders:
    def __init__(self):
        self.encoder_paths = {
            "L": "/sys/bus/counter/devices/counter1/count0/count",  # eQEP1 -> P8_35/33
            "R": "/sys/bus/counter/devices/counter2/count0/count",  # eQEP2 -> P8_12/11
        }
        self.counts_per_rev = 2400.0  # 600 PPR * 4
        self.mock_mode = False
        
        # Configure and enable encoders programmatically
        try:
            for dev in ("counter1", "counter2"):
                dev_dir = f"/sys/bus/counter/devices/{dev}/count0"
                if os.path.isdir(dev_dir):
                    try:
                        with open(os.path.join(dev_dir, "ceiling"), "w") as f:
                            f.write("4294967295")
                    except Exception:
                        pass
                    try:
                        with open(os.path.join(dev_dir, "enable"), "w") as f:
                            f.write("1")
                    except Exception:
                        pass
                else:
                    self.mock_mode = True
        except Exception:
            self.mock_mode = True

        self.prev_counts = {"L": None, "R": None}
        self.last_time = time.monotonic()
        self.rpms = {"L": 0.0, "R": 0.0}

        # Check path existence
        try:
            self.paths_ok = {k: os.path.exists(p) for k, p in self.encoder_paths.items()}
        except Exception:
            self.paths_ok = {"L": False, "R": False}

        if not all(self.paths_ok.values()):
            self.mock_mode = True
            print("[Hardware] eQEP hardware encoder counters missing. Activating Mock/Simulation feedback mode.")
        else:
            print("[Hardware] eQEP hardware encoders initialized successfully.")

    def _read_count(self, key):
        try:
            with open(self.encoder_paths[key], 'r') as f:
                return int(f.read().strip())
        except Exception:
            return None

    def update(self, cmd_rpm_L: float = 0.0, cmd_rpm_R: float = 0.0) -> tuple:
        """Updates and returns the actual (or simulated) wheel RPMs as (RPM_L, RPM_R)."""
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now
        
        if dt <= 0:
            return self.rpms["L"], self.rpms["R"]

        if self.mock_mode:
            # In mock mode, actual RPM matches commanded RPM with a tiny lag (low pass filter)
            self.rpms["L"] = self.rpms["L"] + 0.35 * (cmd_rpm_L - self.rpms["L"])
            self.rpms["R"] = self.rpms["R"] + 0.35 * (cmd_rpm_R - self.rpms["R"])
            return self.rpms["L"], self.rpms["R"]

        for key in ("L", "R"):
            if not self.paths_ok[key]:
                continue
            curr = self._read_count(key)
            if curr is None:
                self.rpms[key] = 0.0
                self.prev_counts[key] = None
                continue
            if self.prev_counts[key] is None:
                self.prev_counts[key] = curr
                self.rpms[key] = 0.0
                continue
                
            delta = curr - self.prev_counts[key]
            self.prev_counts[key] = curr
            
            # Handle 32-bit rollover
            if delta > 2**31:
                delta -= 2**32
            elif delta < -2**31:
                delta += 2**32
                
            # Convert counts/sec to motor shaft RPM
            self.rpms[key] = (delta / (self.counts_per_rev * dt)) * 60.0

        return self.rpms["L"], self.rpms["R"]

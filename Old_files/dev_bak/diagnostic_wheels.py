import time
import sys
import os
from smbus2 import SMBus

# DAC I2C Addresses
DAC_ADDR_L = 0x60
DAC_ADDR_R = 0x61

# Encoder sysfs paths
ENC_PATH_L = "/sys/bus/counter/devices/counter1/count0/count"
ENC_PATH_R = "/sys/bus/counter/devices/counter2/count0/count"

# GPIO paths (main contactor and brake relays)
# Adjust these if your hardware uses different pins
try:
    import Adafruit_BBIO.GPIO as GPIO
    CONTACTOR_PIN = "P9_12" # Example contactor GPIO
    BRAKE_PIN = "P9_15"     # Example brake GPIO
    GPIO.setup(CONTACTOR_PIN, GPIO.OUT)
    GPIO.setup(BRAKE_PIN, GPIO.OUT)
    GPIO_AVAILABLE = True
except Exception as e:
    print(f"GPIO Setup Warning: {e}")
    GPIO_AVAILABLE = False

def write_dac(bus, addr, val):
    # MCP4725 write command: write DAC register
    # val is 12-bit (0 - 4095)
    high_byte = (val >> 4) & 0xFF
    low_byte = (val << 4) & 0xFF
    try:
        bus.write_i2c_block_data(addr, 0x40, [high_byte, low_byte])
        return True
    except Exception as e:
        print(f"DAC write to {hex(addr)} failed: {e}")
        return False

def read_encoders():
    L, R = 0, 0
    try:
        with open(ENC_PATH_L, "r") as f:
            L = int(f.read().strip())
    except Exception:
        pass
    try:
        with open(ENC_PATH_R, "r") as f:
            R = int(f.read().strip())
    except Exception:
        pass
    return L, R

def main():
    print("=== STARTING DIRECT WHEELS DIAGNOSTIC ===")
    
    bus = SMBus(2)
    
    # 1. Energize Contactor & Release Brakes
    if GPIO_AVAILABLE:
        print("Energizing safety contactor...")
        GPIO.output(CONTACTOR_PIN, GPIO.HIGH)
        time.sleep(1.0)
        print("Releasing mechanical brakes...")
        GPIO.output(BRAKE_PIN, GPIO.HIGH)
        time.sleep(1.0)
    else:
        print("Assuming system is energized externally.")
        
    start_L, start_R = read_encoders()
    print(f"Initial Encoder Ticks: L={start_L}, R={start_R}")
    
    print("\nSending command 1300 to BOTH wheels (running for 5 seconds)...")
    
    try:
        for i in range(50): # 5 seconds (100ms steps)
            write_dac(bus, DAC_ADDR_L, 1300)
            write_dac(bus, DAC_ADDR_R, 1300)
            
            time.sleep(0.1)
            curr_L, curr_R = read_encoders()
            print(f"t={i*0.1:.1f}s | Encoders: L={curr_L} (diff={curr_L-start_L}), R={curr_R} (diff={curr_R-start_R})", end='\r')
            sys.stdout.flush()
            
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("\n\nStopping motors...")
        write_dac(bus, DAC_ADDR_L, 0)
        write_dac(bus, DAC_ADDR_R, 0)
        
        if GPIO_AVAILABLE:
            print("Engaging brakes and opening contactor...")
            GPIO.output(BRAKE_PIN, GPIO.LOW)
            time.sleep(0.5)
            GPIO.output(CONTACTOR_PIN, GPIO.LOW)
            
        final_L, final_R = read_encoders()
        print(f"Final Encoder Ticks: L={final_L} (total diff={final_L-start_L}), R={final_R} (total diff={final_R-start_R})")
        print("Diagnostic Complete.")

if __name__ == "__main__":
    main()

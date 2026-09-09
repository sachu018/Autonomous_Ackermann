import time
import os
try:
    import smbus2
    import Adafruit_BBIO.GPIO as GPIO
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False

# Pins
GPIO_CONTACTOR = "P8_14"
GPIO_BRAKE_LEFT = "P8_8"
GPIO_BRAKE_RIGHT = "P8_10"
GPIO_REV_LEFT = "P8_7"
GPIO_REV_RIGHT = "P8_9"

# DACs
DAC_LEFT_ADDR = 0x61
DAC_RIGHT_ADDR = 0x60

def write_dac(bus, addr, value):
    upper = (value >> 4) & 0xFF
    lower = (value << 4) & 0xFF
    try:
        bus.write_i2c_block_data(addr, 0x40, [upper, lower])
    except Exception as e:
        print(f"Error writing to DAC {hex(addr)}: {e}")

def main():
    if not HW_AVAILABLE:
        print("Missing hardware libraries.")
        return

    print("Configuring pinmuxes...")
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
    
    bus = smbus2.SMBus(2)

    try:
        print("Energizing contactor...")
        GPIO.output(GPIO_CONTACTOR, GPIO.HIGH)
        time.sleep(0.5)

        print("Releasing brakes...")
        GPIO.output(GPIO_BRAKE_LEFT, GPIO.LOW)
        GPIO.output(GPIO_BRAKE_RIGHT, GPIO.LOW)
        time.sleep(0.1)

        # TEST LEFT WHEEL (20 seconds)
        print("\n--- STARTING LEFT WHEEL ONLY (20 seconds) ---")
        write_dac(bus, DAC_RIGHT_ADDR, 0)
        write_dac(bus, DAC_LEFT_ADDR, 2000)
        
        for i in range(20):
            print(f"  Left Wheel running... {20-i}s remaining")
            time.sleep(1)
        
        # Stop Left
        write_dac(bus, DAC_LEFT_ADDR, 0)
        time.sleep(2)

        # TEST RIGHT WHEEL (20 seconds)
        print("\n--- STARTING RIGHT WHEEL ONLY (20 seconds) ---")
        write_dac(bus, DAC_LEFT_ADDR, 0)
        write_dac(bus, DAC_RIGHT_ADDR, 2000)
        
        for i in range(20):
            print(f"  Right Wheel running... {20-i}s remaining")
            time.sleep(1)

        # Stop Right
        write_dac(bus, DAC_RIGHT_ADDR, 0)
        time.sleep(1)

    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        print("\nPowering down and engaging brakes...")
        write_dac(bus, DAC_LEFT_ADDR, 0)
        write_dac(bus, DAC_RIGHT_ADDR, 0)
        GPIO.output(GPIO_BRAKE_LEFT, GPIO.HIGH)
        GPIO.output(GPIO_BRAKE_RIGHT, GPIO.HIGH)
        GPIO.output(GPIO_CONTACTOR, GPIO.LOW)
        bus.close()
        GPIO.cleanup()
        print("Done.")

if __name__ == '__main__':
    main()

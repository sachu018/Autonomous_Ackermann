import time
import board
import adafruit_bno055

def main():
    print("Initializing BNO055 I2C interface...")
    try:
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c)
    except Exception as e:
        print(f"Error: Failed to initialize BNO055: {e}")
        return

    print("\\n--- BNO055 Internal Temperature Check ---")
    print("Reading temperature from sensor registers in Celsius...")
    print("Press Ctrl+C to exit.")
    print("-" * 50)

    try:
        while True:
            temp = sensor.temperature
            print(f"Current BNO055 Sensor Temperature: {temp}°C", end='\\r')
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\\nExiting temperature check.")

if __name__ == "__main__":
    main()

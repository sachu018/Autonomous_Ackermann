import Adafruit_BBIO.GPIO as GPIO
import time

REV_LEFT = "P8_7"
REV_RIGHT = "P8_9"

print("Setting up pins...")
GPIO.setup(REV_LEFT, GPIO.OUT)
GPIO.setup(REV_RIGHT, GPIO.OUT)

print("\n*** PINS ARE NOW HIGH ***")
print("You have 15 seconds to measure P8_7 and P8_9 with your multimeter.")
print("They should both read 3.3V.")

GPIO.output(REV_LEFT, GPIO.HIGH)
GPIO.output(REV_RIGHT, GPIO.HIGH)

time.sleep(30)

print("\nTurning pins OFF.")
GPIO.output(REV_LEFT, GPIO.LOW)
GPIO.output(REV_RIGHT, GPIO.LOW)
GPIO.cleanup()
print("Done.")

import Jetson.GPIO as GPIO
import time

# Pin Definitions
led_pin = 33  # Assuming you're using pin 18

# Pin Setup
GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme

GPIO.cleanup()  # Clean up GPIO on CTRL+C exit

GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme

GPIO.setup(led_pin, GPIO.OUT)  # LED pin set as output

print("Press CTRL+C to exit")

try:
    while True:
        GPIO.output(led_pin, GPIO.HIGH)  # Turn LED on
        time.sleep(1)  # Sleep for 1 second
        GPIO.output(led_pin, GPIO.LOW)   # Turn LED off
        time.sleep(1)  # Sleep for 1 second
except KeyboardInterrupt:
    GPIO.cleanup()  # Clean up GPIO on CTRL+C exit


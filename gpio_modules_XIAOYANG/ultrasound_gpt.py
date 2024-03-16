import Jetson.GPIO as GPIO
import time

# Define GPIO pins for trigger and echo
TRIG_PIN = 23
ECHO_PIN = 24

def measure_distance():
    # Send a short pulse to trigger the sensor
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    # Wait for the echo signal to start
    pulse_start = time.time()
    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        if (time.time() - pulse_start) > 1: # Timeout after 1 second
            print("Timeout waiting for echo signal to start")
            return None

    # Wait for the echo signal to end
    pulse_end = time.time()
    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        if (time.time() - pulse_end) > 1: # Timeout after 1 second
            print("Timeout waiting for echo signal to end")
            return None

    # Calculate pulse duration
    pulse_duration = pulse_end - pulse_start


    # Speed of sound in air is approximately 343 meters per second
    # Distance = speed * time / 2 (since the pulse travels to the object and back)
    distance = (pulse_duration * 34300) / 2 # Distance in centimeters

    return distance

try:
    # Set up GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)

    while True:
        dist = measure_distance()
        if dist is not None:
            print("Distance:", dist, "cm")
            time.sleep(1) # Repeat measurement every 1 second

except KeyboardInterrupt:
    print("Measurement stopped by user")
finally:
    GPIO.cleanup()
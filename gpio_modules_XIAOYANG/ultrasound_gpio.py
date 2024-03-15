import Jetson.GPIO as GPIO
import time

# GPIO pins
TRIG_PIN = 23
ECHO_PIN = 24

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def get_distance():
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound = 343m/s = 34300cm/s, Distance = time * speed
    distance = round(distance, 2)
    return distance

try:
    while True:
        dist = get_distance()
        print("Distance:", dist, "cm")
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()
import Jetson.GPIO as GPIO
# from gpiozero import DistanceSensor
# from gpiozero.pins.native import NativeFactory
# import gpiozero.devices
import time

# gpiozero.devices.Device.pin_factory = NativeFactory()
# gpiozero.devices.pi_info = None

# Pin Definitions
led_pin = 33  # Assuming you're using pin 18
# GPIO pins
TRIG_PIN = 23 #was 23
ECHO_PIN = 24 #was 24
test_PIN = 26 
empty_PIN = 22

# Pin Setup
GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
GPIO.cleanup()  # Clean up GPIO on CTRL+C exit
GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
GPIO.setup(led_pin, GPIO.OUT)  # LED pin set as output

# Setup GPIO
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(test_PIN, GPIO.IN)
GPIO.setup(empty_PIN, GPIO.IN)


# sensor = DistanceSensor(echo=24, trigger=23)

print("Press CTRL+C to exit")

def led_blink():
    GPIO.output(led_pin, GPIO.HIGH)  # Turn LED on
    time.sleep(1)  # Sleep for 1 second
    GPIO.output(led_pin, GPIO.LOW)   # Turn LED off
    time.sleep(1)  # Sleep for 1 second


def get_distance():
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.0001)
    GPIO.output(TRIG_PIN, False)

    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
        print('0')

    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
        print('1')

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound = 343m/s = 34300cm/s, Distance = time * speed
    distance = round(distance, 2)
    return distance


try:
    while True:
        led_blink()
        print('LED Blinked')
        dist = get_distance()
        print('Distance: ', dist)
        # print('TEST PIN: ', GPIO.input(test_PIN))
        # print('EMPTY PIN: ', GPIO.input(empty_PIN))
        time.sleep(0.1)
except KeyboardInterrupt:
    GPIO.cleanup()  # Clean up GPIO on CTRL+C exit

# try:
#     while True:
#         # dist = get_distance()
#         # print("Distance:", dist, "cm")
#         GPIO.output(TRIG_PIN, True)
#         print(GPIO.input(ECHO_PIN))

#         # time.sleep(1)

# except KeyboardInterrupt:
#     GPIO.cleanup()
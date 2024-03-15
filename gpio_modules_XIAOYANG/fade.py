import Jetson.GPIO as GPIO
import time

led_pin = 33

GPIO.setmode(GPIO.BOARD)
pwm_led = GPIO.PWM(33,500)
pwm_led.start(1)
while True:
    pwm_led.ChangeDutyCycle(0)
    print(0)
    time.sleep(1)
    pwm_led.ChangeDutyCycle(100)
    print(1)
    time.sleep(1)


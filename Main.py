from motor_drive_modules.Event_Handler import Event_Handler
from motor_drive_modules.Chassis import Chassis
from motor_drive_modules.ultrasound_via_arduino import Ultrasound
from motor_drive_modules.RosmasterBoard import Rosmaster
import time
import serial



# Setup
bot = Rosmaster()
bot.create_receive_threading()
bot.set_auto_report_state(enable = True)
bot.clear_auto_report_data()

print('START')
print('Program Start with a sleep of 15 seconds')
time.sleep(15)
bot.set_beep(100)
chassis = Chassis()
ultrasound = Ultrasound()
ser = serial.Serial("/dev/ttyACM0", 115200)

print("Start Measure")
yaw_rate = chassis.measure_stationary_yaw_drift_rate(15)
bot.set_beep(100)
print("Yaw Rate: {}".format(yaw_rate))
print('IMU gloabl start: {}'.format(chassis.imu_global_start))

# pseudo function for getting target
def get_target():
    return (-1.23, -3.45, 20) # sx, y, angle


# Main Loop
while True:
    chassis.event_handler.empty_events()

    while True:
        Chassis.forward()
        if chassis.event_handler.reset_flag and chassis.event_handler.timeout_flag:
            break
        time.sleep(1)
        Chassis.right()
        if chassis.event_handler.reset_flag and chassis.event_handler.timeout_flag:
            break
        time.sleep(1)
        
        # ....
    while chassis.event_handler.timeout_flag:
        pass

    # return loop
    while (abs(get_target[0]) > 10) and (abs(get_target[1]) > 15):
        chassis.forward()
        chassis.right()
        if get_target[2] > 10:
            chassis.turn(get_target[2])


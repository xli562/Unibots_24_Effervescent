from motor_drive_modules.Event_Handler import Event_Handler
from motor_drive_modules.Chassis import *
from motor_drive_modules.RosmasterBoard import Rosmaster
import time
import serial



# Setup
bot = Rosmaster()
bot.create_receive_threading()
bot.set_auto_report_state(enable = True)
bot.clear_auto_report_data()

ultrasound = Ultrasound()
lidar = Lidar()
intake = Intake()
event_handler = Event_Handler()


print('START')
print('Program Start with a sleep of 15 seconds')
time.sleep(15)
bot.set_beep(100)
chassis = Chassis(ultrasound, lidar, intake, event_handler)
ser = serial.Serial("/dev/ttyACM0", 115200)

print("Start Measure")
yaw_rate = chassis.measure_stationary_yaw_drift_rate(15)
bot.set_beep(100)
print("Yaw Rate: {}".format(yaw_rate))
print('IMU gloabl start: {}'.format(chassis.imu_global_start))

# Main Loop
while True:
    chassis.event_handler.empty_events()
    chassis.event_handler.iteration_start_time = time.time()

    while chassis.event_handler.timeout_flag:
        Chassis.forward()
        if chassis.event_handler.reset_flag and chassis.event_handler.timeout_flag:
            break
        time.sleep(0.5)
        Chassis.right()
        if chassis.event_handler.reset_flag and chassis.event_handler.timeout_flag:
            break

    chassis.revert_orientation()
        
        
    # return loop
    while (abs(chassis.find_base[0] > 45)) or (abs(chassis.find_base[1]) > 45):
        if chassis.find_base[0] > 45:
            chassis.forward()
        if chassis.find_base[1] > 45:
            chassis.right()
        


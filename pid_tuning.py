# from RosmasterBoard import Rosmaster
from motor_drive_modules.Chassis import *
import time
import serial

# bot = Rosmaster()
# bot.create_receive_threading()
# # Enable auto data sending every 40ms.
# bot.set_auto_report_state(enable = True)
# # Clear cache sent from the Rosmaster board
# bot.clear_auto_report_data()

ser = serial.Serial("/dev/ttyACM0", 115200)

ultrasound = Ultrasound(ser=ser)
event_handler = Event_Handler(ser=ser)
lidar = Lidar()
intake = Intake()
buzzer = Buzzer()

print('Program Start with a sleep of 15 seconds')
#time.sleep(15)
bot.set_beep(100)
chassis = Chassis(ultrasound, lidar, intake, event_handler, buzzer)
# ultrasound = Ultrasound()
# ser = serial.Serial("/dev/ttyACM1", 115200)

print("Start Measure")
yaw_rate = chassis.measure_stationary_yaw_drift_rate(5)
bot.set_beep(100)
print("Yaw Rate: {}".format(yaw_rate))
print('IMU gloabl start: {}'.format(chassis.imu_global_start))

try:
    chassis.forward()
    #time.sleep(0.2)
    #chassis.right()
    #time.sleep(0.2)
    #chassis.back()
    #time.sleep(0.2)
    #chassis.left()
    # time.sleep(0.2)
except Exception as e:
    print(e)
    del bot
    del ultrasound
    del event_handler
    del lidar
    del intake
    del buzzer
    print('bot instance deleted')
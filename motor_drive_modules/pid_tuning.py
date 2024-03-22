from RosmasterBoard import Rosmaster
from ultrasound_via_arduino import Ultrasound
from Chassis import Chassis
import time
import serial

bot = Rosmaster()
bot.create_receive_threading()
# Enable auto data sending every 40ms.
bot.set_auto_report_state(enable = True)
# Clear cache sent from the Rosmaster board
bot.clear_auto_report_data()

print('Program Start with a sleep of 15 seconds')
time.sleep(15)
bot.set_beep(100)
chassis = Chassis()
ultrasound = Ultrasound()
ser = serial.Serial("/dev/ttyACM0", 115200)

print("Start Measure")
yaw_rate = chassis.measure_statixonary_yaw_drift_rate(15)
bot.set_beep(100)
print("Yaw Rate: {}".format(yaw_rate))
print('IMU gloabl start: {}'.format(chassis.imu_global_start))

# time.sleep(0.2)
#chassis.forward()
#time.sleep(0.2)
chassis.right()
#time.sleep(0.2)
#chassis.back()
#time.sleep(0.2)
#chassis.left()
# time.sleep(0.2)
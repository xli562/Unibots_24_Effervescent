from Chassis import Intake
import sys
import subprocess
import time
import serial
from Chassis import Chassis
from RosmasterBoard import Rosmaster
from ultrasound_via_arduino import Ultrasound

ser = serial.Serial("/dev/Arduino", 115200)


# Rerun the current file
def ROBOT_RESET():
    print("Restarting...")
    python = '/home/eff/Desktop/Unibots_24_Effervescent/renv/bin/python3.10'
    subprocess.call([python, "test.py"])
    sys.exit()

def check_restart():
    print('restart_point_1')
    try:
        if ser.in_waiting > 0:
            data_str = ser.readline().strip().decode('utf-8')
            print('Received Data: {}'.format(data_str))
            readings = data_str.split('!')
            for reading in readings:
                if reading == "Restart":  # Command to restart - sent when restart button of Arduino is pressed
                    print('RESTARTING THE ROBOT')
                    ROBOT_RESET()
    except Exception as e:
        print(e)

    
print('Start')
index = 0
chassis = Chassis()
bot = Rosmaster()
bot.create_receive_threading()
# Enable auto data sending every 40ms.
bot.set_auto_report_state(enable = True)
# Clear cache sent from the Rosmaster board
bot.clear_auto_report_data()


print('Program Start with a sleep of 15 seconds')
time.sleep(5)
bot.set_beep(100)
chassis = Chassis()
ultrasound = Ultrasound()
ser = serial.Serial("/dev/Arduino", 115200)

print("Start Measure")
yaw_rate = chassis.measure_stationary_yaw_drift_rate(15)
bot.set_beep(100)
print("Yaw Rate: {}".format(yaw_rate))
print('IMU gloabl start: {}'.format(chassis.imu_init_angle_offset))


while True:

    # check_restart()
    chassis.forward()
    print(index)
    index += 1
    time.sleep(0.1)




# intake = Intake()
# intake.set_free_drive()
# print('Sert zero power')
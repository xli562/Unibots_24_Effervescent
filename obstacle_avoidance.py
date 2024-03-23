from modules.Chassis import *
import time
import serial
import subprocess
import sys

# bot = Rosmaster()
# bot.create_receive_threading()
# # Enable auto data sending every 40ms.
# bot.set_auto_report_state(enable = True)
# # Clear cache sent from the Rosmaster board
# bot.clear_auto_report_data()

ser = serial.Serial("/dev/Arduino", 115200)

# Rerun the current file
def ROBOT_RESET():
    print("Restarting...")
    python = sys.executable
    subprocess.call([python, "obstacle_avoidance.py"])
    sys.exit()

def check_restart():
    data_str = ser.readline().strip().decode('utf-8')
    readings = data_str.split('!')
    for reading in readings:
        if reading == "Restart":  # Command to restart - sent when restart button of Arduino is pressed
            print('RESTARTING THE ROBOT')
            ROBOT_RESET()


   
# while True:
#     # 超声波采数据
#     check_restart()
#     ultrasound.receive_distances()
#     distances = ultrasound.get_distances()  
#     print("Distances:", distances)
#     print("Obstacles at directions: {}".format(ultrasound.check_obstacle))

ultrasound = Ultrasound(arduino_ser = ser)
lidar = Lidar()
intake = Intake()
event_handler = Event_Handler(ser = ser)
buzzer = Buzzer()


print('START')
print('Program Start with a sleep of 15 seconds')
chassis = Chassis(ultrasound, lidar, intake, event_handler, buzzer)

time.sleep(2)
bot.set_beep(100)


print("Start Measure")
yaw_rate = chassis.measure_stationary_yaw_drift_rate(5)
bot.set_beep(100)
print("Yaw Rate: {}".format(yaw_rate))
print('IMU gloabl start: {}'.format(chassis.imu_init_angle_offset))

# while True:
    #check_restart()
chassis.forward()
print('Obstacle Forward')
time.sleep(5)
chassis.backward()
print('Obstacle Backward')
time.sleep(0.5)
# chassis.right()
# print('Obstacle Right')
# time.sleep(0.5)
# chassis.left()
# print('Obstacle Left')
# time.sleep(0.5)


    # print('After Forward')
    # ultrasound.receive_distances()
    # distances = ultrasound.get_distances()  
    # # print("Distances:", distances)
    # # print("Obstacles at directions: {}".format(ultrasound.check_obstacle))

    # if ultrasound.object_front:
    #     print("Object at Front")
    #     chassis.reset()
    #     break
        # if ultrasound.rugby_left():
        #     chassis.grab_rugby() #TODO: Define and write the function grab_rugby
    # while True:
    
    #     check_restart()
    #     chassis.right()
    #     if ultrasound.object_right:
    #         print("Object Right")
    #         chassis.reset()
    #         break
    

time.sleep(0.02)  

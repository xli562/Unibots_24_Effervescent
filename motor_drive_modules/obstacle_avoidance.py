from imu_read import Chassis
from ultrasound_via_arduino import Ultrasound
import time
import serial
import subprocess
import sys

chassis = Chassis()
ultrasound = Ultrasound()
ser = serial.Serial("/dev/ttyUSB0", 115200)

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
            ROBOT_RESET()


   

while True:
    # 超声波采数据
    check_restart()
    ultrasound.receive_distances()
    distances = ultrasound.get_distances()  
    print("Distances:", distances)
    print("Obstacles at directions: {}".format(ultrasound.check_obstacle))

    while True:
        check_restart()
        chassis.straight_forward()
        if ultrasound.object_front():
            print("Object at Front")
            chassis.reset()
            break
        # if ultrasound.rugby_left():
        #     chassis.grab_rugby() #TODO: Define and write the function grab_rugby
    while True:
        check_restart()
        chassis.right()
        if ultrasound.object_right():
            print("Object Right")
            chassis.reset()
            break
    

    time.sleep(0.02)  

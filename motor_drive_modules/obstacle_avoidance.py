from imu_read import Chassis
from ultrasound_via_arduino import Ultrasound
import time

chassis = Chassis()
ultrasound = Ultrasound()

while True:
    # 超声波采数据
    ultrasound.receive_distances()
    distances = ultrasound.get_distances()  
    print("Distances:", distances)
    print("Obstacles at directions: {}".format(ultrasound.check_obstacle))

    while True:
        chassis.straight_forward()
        if ultrasound.object_front():
            print("Object at Front")
            chassis.reset()
            break
        if ultrasound.rugby_left():
            chassis.grab_rugby() #TODO: Define and write the function grab_rugby
    while True:
        chassis.right()
        if ultrasound.object_right():
            print("Object Right")
            chassis.reset()
            break
    

    
    time.sleep(0.02)  

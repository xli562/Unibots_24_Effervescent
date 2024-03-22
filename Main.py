from motor_drive_modules.Event_Handler import Event_Handler
from motor_drive_modules.Chassis import *
import time
import serial



# Setup
# bot = Rosmaster()
# bot.create_receive_threading()
# bot.set_auto_report_state(enable = True)
# bot.clear_auto_report_data()

ultrasound = Ultrasound()
lidar = Lidar()
intake = Intake()
event_handler = Event_Handler()
buzzer = Buzzer()


print('START')
print('Program Start with a sleep of 15 seconds')
chassis = Chassis(ultrasound, lidar, intake, event_handler, buzzer)
chassis.stop()
# bot.set_car_motion(0,0,0)
# intake.set_free_drive()

time.sleep(15)
bot.set_beep(100)
# chassis = Chassis(ultrasound, lidar, intake, event_handler, buzzer)
ser = serial.Serial("/dev/ttyACM0", 115200)

print("Start Measure")
yaw_rate = chassis.measure_stationary_yaw_drift_rate(15)
bot.set_beep(100)
print("Yaw Rate: {}".format(yaw_rate))
print('IMU gloabl start: {}'.format(chassis.imu_global_start))

def move(direction, condition, duration = None):
    yaw_start = chassis.get_yaw_calibrated()
    if direction == 'f':
        # yaw_start = chassis.get_yaw_calibrated()
        pid = PID(0.5,0,0.1, setpoint=yaw_start)
        chassis.vx = 0.2
        chassis.vy = 0
    elif direction == 'b':
        # yaw_start = chassis.get_yaw_calibrated()
        pid = PID(0.5,0,0.1, setpoint=yaw_start)
        chassis.vx = -0.2
        chassis.vy = 0
    elif direction == 'l':
        # yaw_start = chassis.get_yaw_calibrated()
        pid = PID(0.2, -0.5, 0.01, setpoint=yaw_start)
        chassis.vx = 0
        chassis.vy = -0.2
    elif direction == 'r':
        # yaw_start = chassis.get_yaw_calibrated()
        pid = PID(0.05, 0, 0.05, setpoint=yaw_start)
        chassis.vx = 0
        chassis.vy = 0.2


    if duration is None:
        while not(condition):
            chassis.ultrasound.receive_distances()
            distances = chassis.ultrasound.get_distances()  
            print("Distances:", distances)
            print("Obstacles at directions: {}".format(chassis.ultrasound.check_obstacle))
            chassis.action(pid, yaw_start)
    else:
        start = time.time()
        end = time.time()
        while (end - start < duration) and not(condition):
            end = time.time()
            chassis.ultrasound.receive_distances()
            distances = chassis.ultrasound.get_distances()  
            print("Distances:", distances)
            print("Obstacles at directions: {}".format(chassis.ultrasound.check_obstacle))
            chassis.action(pid, yaw_start)

    chassis.stop()
    

def turn(angle): # -ve for right, +ve for left
    yaw_start = chassis.get_yaw_calibrated()
    pid_turn = PID(0.07, 0, 0.03, setpoint = yaw_start + angle)
    chassis.vx = 0
    chassis.vy = 0
    yaw = yaw_start
    previous_yaw = yaw_start
    while abs(yaw_start + angle - yaw) > 1:
        try:
            yaw = chassis.get_yaw_calibrated()
            if (yaw-previous_yaw) > 300:
                yaw -= 360
            elif (yaw-previous_yaw) < -300:
                yaw += 360
            previous_yaw = yaw
            error = yaw_start + angle - yaw 
            control = pid_turn(yaw)
            chassis.vz = max(-10, min(control, 10))
            print("Error: {}, Control: {}, Vz: {}".format(error, control, chassis.vz))
            bot.set_car_motion(chassis.vx, chassis.vy, chassis.vz)
            chassis.intake.set_eat_power()
            time.sleep(0.1)
        except KeyboardInterrupt:
            chassis.stop()
            break

    ######
    if (angle < 0): #Assuming turns can only be +90 or -90
        chassis.turn_num -= 1  
    else:
        chassis.turn_num += 1

    chassis.stop()    



# Main Loop
while True:
    chassis.event_handler.empty_events()
    chassis.event_handler.iteration_start_time = time.time()

    # Moving right a little bit to start with
    if not(chassis.event_handler.timeout_flag):
        move('r',chassis.ultrasound.object_right, duration = 2) # 2 seconds move right

    # Going out, Searching & Grabbing
    while chassis.event_handler.timeout_flag:
        # Chassis.forward()
        move('f',chassis.ultrasound.object_front)

        if chassis.event_handler.reset_flag and chassis.event_handler.timeout_flag:
            break
        time.sleep(0.5)

        # Chassis.right()
        move('r',chassis.ultrasound.object_right)
        if chassis.event_handler.reset_flag and chassis.event_handler.timeout_flag:
            break

    if chassis.event_handler.timeout_flag:
        # Turn to correct orientation for return-to-base to begin
        chassis.revert_orientation()
        
        
    # # return loop
    # while (abs(chassis.find_base[0] > 45)) or (abs(chassis.find_base[1]) > 45):
    #     if chassis.find_base[0] > 45:
    #         chassis.forward()
    #     if chassis.find_base[1] > 45:
    #         chassis.right()
       


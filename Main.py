from modules.Chassis import *
import time
import serial

# If we are testing the robot stationary,
# shorten the wait times for yaw calibration.
do_stationary_test = False


buzzer = Buzzer()
servo = Servo()
intake = Intake()
lidar = Lidar(buzzer.beep_pattern)
arduino_board = Arduino(buzzer.beep_pattern)
ultrasound = Ultrasound(arduino_board.ultrasound_new_reading_available,
                        arduino_board.get_last_ultrasound_readings)
event_handler = EventHandler(arduino_board.received_reset)
chassis = Chassis(buzzer.beep_pattern, intake, lidar, event_handler)
chassis.stop()


print('START')
if do_stationary_test:
    print('Program Start without sleep')
else:
    print('Program Start with a sleep of 15 seconds')
    time.sleep(3)
bot.set_beep(100)


print("Start Measure")
imu_calibration_time = 5 if do_stationary_test else 20
yaw_rate = chassis.measure_stationary_yaw_drift_rate(imu_calibration_time)
bot.set_beep(100)
chassis.imu_init_angle_offset = chassis.get_yaw_calibrated()
print(f'Yaw Rate: {yaw_rate}')
print(f'IMU gloabl start: {chassis.imu_init_angle_offset}')

def move(direction:str, duration=None, 
         check_obstacle=ultrasound.check_obstacle): 
    """
    :param check_obstacle: the getter function to retrieve stoping 
        condition from ultrasound """
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
    else:
        raise Exception(f'Direction has to be "f", "b", "l" or "r", got {direction}.')

    if duration is None:
        while not check_obstacle(direction) and not(chassis.event_handler.reset_flag) and not(chassis.event_handler.timeout_flag): ####(Max)#### Added the chassis.event_handler.reset_flag
            # print(ultrasound.check_all_obstacles())
            chassis.event_handler.check_reset()
            chassis.event_handler.check_timeout()
            if chassis.event_handler.reset_flag:
                break
            if chassis.event_handler.timeout_flag:
                break
            # print(direction) 
            # print(f'Obstacles at directions: {chassis._ultrasound.check_all_obstacles()}')
            chassis.action(pid, yaw_start)
    else:
        start = time.time()
        end = time.time()
        while (end - start < duration) and not(check_obstacle(direction)) and not(chassis.event_handler.reset_flag) and not(chassis.event_handler.timeout_flag):  ####(Max)#### Added the chassis.event_handler.reset_flag
            end = time.time()
            chassis.action(pid, yaw_start)

    chassis.stop()
    

def turn(angle): # -ve for clockwise, +ve for anticlockwise
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
            print(f'Error: {error}, Control: {control}, Vz: {chassis}')
            bot.set_car_motion(chassis.vx, chassis.vy, chassis.vz)
            chassis._intake.eat()
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
    if not chassis.event_handler.timeout_flag:
        move('r', duration = 5) # 2 seconds move right

    # Going out, searching & grabbing, stop when timeout
    while not chassis.event_handler.timeout_flag:
        # Move forward
        move('f')
        if chassis.event_handler.reset_flag or chassis.event_handler.timeout_flag:
            chassis.stop()
            break

        # Move to the right
        move('r')
        if chassis.event_handler.reset_flag or chassis.event_handler.timeout_flag:
            chassis.stop()
            break
        chassis.event_handler.check_timeout()


    # Turn to correct orientation for return-to-base to begin
    ####(Max)####
    if chassis.event_handler.timeout_flag and not(chassis.event_handler.reset_flag):

        print('Returning to scoring area')
        buzzer.beep_pattern('...')
        time.sleep(5)
        #chassis.revert_orientation()
    
    if (chassis.event_handler.reset_flag):
        time.sleep(10)

        
    # # return loop
    # x_base = 100
    # y_base = 100
    # while x_base > 45 or y_base > 45:
    #     if y_base > 45:
    #         move('f')
    #         base = chassis.find_base()
    #         x_base, y_base = base[0], base[1]
    #     if x_base > 45:
    #         move('r')
    #         base = chassis.find_base()
    #         x_base, y_base = base[0], base[1]
       


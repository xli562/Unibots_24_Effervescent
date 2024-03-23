from modules.Chassis import *
import time
import serial

# bot = Rosmaster()
# bot.create_receive_threading()
# # Enable auto data sending every 40ms.
# bot.set_auto_report_state(enable = True)
# # Clear cache sent from the Rosmaster board
# bot.clear_auto_report_data()

ser = serial.Serial("/dev/Arduino", 115200)

ultrasound = Arduino(arduino_ser=ser)
event_handler = EventHandler(ser=ser)
lidar = Lidar()
intake = Intake()
buzzer = Buzzer()

def move(direction, duration=None): # getter represents the getter function to retrieve stoping condition from ultrasound
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
        while not(chassis.event_handler.reset_flag): ####(Max)#### Added the chassis.event_handler.reset_flag
            chassis.ultrasound.receive_distances()
            chassis.event_handler.check_reset()
            if chassis.event_handler.reset_flag:
                break
            print(direction) 
            print(f'Obstacles at directions: {chassis.ultrasound.check_obstacle}')
            chassis.action(pid, yaw_start)
    else:
        start = time.time()
        end = time.time()
        while (end - start < duration) and not(chassis.event_handler.reset_flag):  ####(Max)#### Added the chassis.event_handler.reset_flag
            end = time.time()
            chassis.ultrasound.receive_distances()
            chassis.action(pid, yaw_start)

    chassis.stop()
    


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
print('IMU gloabl start: {}'.format(chassis.imu_init_angle_offset))

try:
    move('f')
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
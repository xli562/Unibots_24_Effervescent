from modules.Event_Handler import Event_Handler
from modules.Chassis import *
import time
import serial

ser = serial.Serial("/dev/Arduino", 115200)

ultrasound = Ultrasound(arduino_ser = ser)
lidar = Lidar()
intake = Intake()
event_handler = Event_Handler(ser = ser)
buzzer = Buzzer()


print('START')
print('Program Start with a sleep of 15 seconds')
chassis = Chassis(ultrasound, lidar, intake, event_handler, buzzer)
chassis.stop()

time.sleep(2)
bot.set_beep(100)

print("Start Measure")
yaw_rate = chassis.measure_stationary_yaw_drift_rate(5)
bot.set_beep(100)
print("Yaw Rate: {}".format(yaw_rate))
print('IMU gloabl start: {}'.format(chassis.imu_init_angle_offset))


index_out = 0
index_in = 0

while True:
    print("Out Index: {}".format(index_out))
    index_out += 1
    inndex_in = 0
    event_handler.empty_events()
    while not(event_handler.reset_flag):
        chassis.forward()
        print("Ultrasound Front Triggered")
        print("Index In:{}".format(index_in))
        chassis.backward()
        print("Ultrasound Back Triggered")
        print("Index In:{}".format(index_in))
        chassis.left()
        print("Ultrasound Left Triggered")
        print("Index In:{}".format(index_in))
        chassis.right()
        print("Ultrasound Right Triggered")
        print("Index In:{}".format(index_in))
    print("Break from the main while loop)")

# def move(direction, condition, duration = None):
#     yaw_start = chassis.get_yaw_calibrated()
#     if direction == 'f':
#         # yaw_start = chassis.get_yaw_calibrated()
#         pid = PID(0.5,0,0.1, setpoint=yaw_start)
#         chassis.vx = 0.2
#         chassis.vy = 0
#     elif direction == 'b':
#         # yaw_start = chassis.get_yaw_calibrated()
#         pid = PID(0.5,0,0.1, setpoint=yaw_start)
#         chassis.vx = -0.2
#         chassis.vy = 0
#     elif direction == 'l':
#         # yaw_start = chassis.get_yaw_calibrated()
#         pid = PID(0.2, -0.5, 0.01, setpoint=yaw_start)
#         chassis.vx = 0
#         chassis.vy = -0.2
#     elif direction == 'r':
#         # yaw_start = chassis.get_yaw_calibrated()
#         pid = PID(0.05, 0, 0.05, setpoint=yaw_start)
#         chassis.vx = 0
#         chassis.vy = 0.2


#     if duration is None:
#         while not(condition):
#             chassis.ultrasound.receive_distances()
#             distances = chassis.ultrasound.get_distances()  
#             print("Distances:", distances)
#             print("Obstacles at directions: {}".format(chassis.ultrasound.check_obstacle))
#             chassis.action(pid, yaw_start)
#     else:
#         start = time.time()
#         end = time.time()
#         while (end - start < duration) and not(condition):
#             end = time.time()
#             chassis.ultrasound.receive_distances()
#             distances = chassis.ultrasound.get_distances()  
#             print("Distances:", distances)
#             print("Obstacles at directions: {}".format(chassis.ultrasound.check_obstacle))
#             chassis.action(pid, yaw_start)

#     chassis.stop()
    

# def turn(angle): # -ve for right, +ve for left
#     yaw_start = chassis.get_yaw_calibrated()
#     pid_turn = PID(0.07, 0, 0.03, setpoint = yaw_start + angle)
#     chassis.vx = 0
#     chassis.vy = 0
#     yaw = yaw_start
#     previous_yaw = yaw_start
#     while abs(yaw_start + angle - yaw) > 1:
#         try:
#             yaw = chassis.get_yaw_calibrated()
#             if (yaw-previous_yaw) > 300:
#                 yaw -= 360
#             elif (yaw-previous_yaw) < -300:
#                 yaw += 360
#             previous_yaw = yaw
#             error = yaw_start + angle - yaw 
#             control = pid_turn(yaw)
#             chassis.vz = max(-10, min(control, 10))
#             print("Error: {}, Control: {}, Vz: {}".format(error, control, chassis.vz))
#             bot.set_car_motion(chassis.vx, chassis.vy, chassis.vz)
#             chassis.intake.set_eat_power()
#             time.sleep(0.1)
#         except KeyboardInterrupt:
#             chassis.stop()
#             break

#     ######
#     if (angle < 0): #Assuming turns can only be +90 or -90
#         chassis.turn_num -= 1  
#     else:
#         chassis.turn_num += 1

#     chassis.stop()    



# while True:
#     print("Out Index: {}".format(index_out))
#     index_out += 1
#     inndex_in = 0
#     event_handler.empty_events()
#     while not(event_handler.reset_flag):
#         move('f',chassis.ultrasound.object_front)
#         print("Ultrasound Front Triggered")
#         print("Index In:{}".format(index_in))
#         move('b',chassis.ultrasound.object_back)
#         print("Ultrasound Back Triggered")
#         print("Index In:{}".format(index_in))
#         move('l',chassis.ultrasound.object_left)
#         print("Ultrasound Left Triggered")
#         print("Index In:{}".format(index_in))
#         move('r',chassis.ultrasound.object_right)
#         print("Ultrasound Right Triggered")
#         print("Index In:{}".format(index_in))
#     print("Break from the main while loop)")



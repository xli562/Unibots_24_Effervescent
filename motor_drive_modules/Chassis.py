from PIDController import PIDController
import time
from RosmasterBoard import Rosmaster
import math
from decimal import Decimal, getcontext
from simple_pid import PID
from ultrasound_via_arduino import Ultrasound
import sys
import subprocess
import serial
from Event_Handler import Event_Handler


getcontext().prec = 28

# global array to store motor velocity
# N.B. Actual velocity of the wheel is this times a constant
motor_velocity_array = [0,0,0,0]

bot = Rosmaster()
bot.create_receive_threading()
# Enable auto data sending every 40ms.
bot.set_auto_report_state(enable = True)
# Clear cache sent from the Rosmaster board
bot.clear_auto_report_data()


def exponential_moving_average(new_value, previous_ema, alpha=0.1):
    return alpha * new_value + (1 - alpha) * previous_ema

ser = serial.Serial("/dev/ttyACM0", 115200)
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


class Motor:
    def __init__(self, port):
        """ Initializes the motor on the given port. """
        self._port = port    # port of the motor
        self.output_power = 0   # output pwr, determines pwm ratio
        self.position = 0   # position of encoder
        self.tolerance = 10
        self.vel_tolerance = 0.1
        # PID parameters
        self.kP = 1
        self.kI = 0
        self.kD = 0
        self.vel_kP = 1
        self.vel_kI = 0
        self.vel_kD = 0
        self.position = self.update_position()

    def set_duty_cycle(self, power, reverse=False):
        """ Sets the motor's drive power, with an option to reverse direction. """
        """ Power range from -100 to 100 """
        self.output_power = -power if reverse else power
        motor_velocity_array[self._port - 1] = self.output_power
        bot.set_motor(motor_velocity_array[0], motor_velocity_array[1], motor_velocity_array[2], motor_velocity_array[3])

    def update_position(self):
        """ Updates the motor's position (acquiring from bot) """
        encoder_readings = bot.get_motor_encoder()
        self.position = encoder_readings[self._port - 1]
        time.sleep(0.001)
        return self.position

    def set_free_drive(self):
        """Sets the motor to free drive mode."""
        self.set_duty_cycle(0)

    def set_pid_coefficients(self, kP, kI, kD):
        self.kP = kP
        self.kI = kI
        self.kD = kD

    def set_vel_pid_coefficients(self, vel_kP, vel_kI, vel_kD):
        self.vel_kP = vel_kP
        self.vel_kI = vel_kI
        self.vel_kD = vel_kD

    def set_position(self, target_position):
        """ Moves the motor to a specific position using PID control. """
        self.update_position()
        position_pid = PIDController(self.kP, self.kI, self.kD)
        while abs(target_position - self.position) >= self.tolerance:
            self.update_position()  # Update positional readings of the motor
            pid_output = position_pid.calculate(self.position, target_position)
            # Ensure control effort is within [-100, 100] range
            control_effort = max(min(pid_output, 100), -100)
            self.set_duty_cycle(control_effort)
            print(">>>>>>", target_position - self.position)
            time.sleep(0.001)
        self.set_duty_cycle(0)  # Stop the motor once target position is within tolerance

    def set_velocity(self, target_vel):
        """Moves the motor at a specific velocity using PID control."""
        previous_position = Decimal(self.update_position())
        previous_time = Decimal(time.time())
        pid_output_ema = Decimal(0)  # Initialize outside the loop
        velocity_pid = PIDController(self.vel_kP, self.vel_kI, self.vel_kD)

        while True:
            time.sleep(0.01)  # Control the loop rate to be more consistent
            current_time = Decimal(time.time())
            elapsed_time = max(current_time - previous_time, Decimal('0.001'))  # Avoid division by very small number

            current_position = Decimal(self.update_position())
            ticks_per_revolution = Decimal('1320.0')
            # Convert ticks to radians per second
            current_vel = (current_position - previous_position) * Decimal('2.0') * Decimal(math.pi) / (ticks_per_revolution * elapsed_time)

            if abs(target_vel - Decimal(current_vel)) < self.vel_tolerance:
                self.set_duty_cycle(0)  # Stop the motor
                break

            pid_output = velocity_pid.calculate(current_vel, target_vel)
            pid_output_ema = exponential_moving_average(Decimal(pid_output), pid_output_ema, alpha=Decimal('0.1'))
            control_effort = max(min(pid_output_ema, Decimal('100')), Decimal('-100'))
            self.set_duty_cycle(float(control_effort))

            print(">>>>>> Current Vel", current_vel)

            previous_position = current_position
            previous_time = current_time


class Servo:
    """ The class to represent the gripper's servo. """
    
    def __init__(self, port):
        self._port = port
        self._position = 0   # target angular position of servo
        self._grip_position = 150
        self._release_position = 27
        self._min_responsive_pos = 23
        self._max_responsive_pos = 160
    
    def set_position(self, degrees:int):
        """ Rotate the servo to specified angular position. 
        Position must be an integer, as specified by Rosmaster_Lib"""
        # Limit the input between min and max angles, beyond which the IDP servos will not move.
        degrees = max(self._min_responsive_pos, min(self._max_responsive_pos, degrees))
        bot.set_pwm_servo(1, degrees)
        self._position = degrees

    def get_position(self) -> int:
        return self._position

    def grip(self):
        """ Grip a mini rugby. """
        ''' TODO: Calibration '''
        self.set_position(self._grip_position)

    def release(self):
        """ Release a mini rugby. """
        ''' TODO: Calibration '''
        self.set_position(self._release_position)

    def test_run():
        """ Tests basic servo functions """
        servo = Servo(1)
        print(servo.get_position())
        while 1:
            servo.grip()
            time.sleep(1)
            servo.release()
            time.sleep(1)


class Intake:
    """ Class to drive the intake motor.
    N.B. motor stops when power is between [-5,5],
        and small increments in power might not make any actual difference,
        as mapping returns int, not float. For example, 
        power = 10 and power = 12 might not make an actual difference 
        to the output power."""

    def __init__(self, port=2):
        """ Initializes the intake motor on the given port. """
        self._port = port    # output port to arduino
        self._eat_power = 100   # intaking pwr, determines pwm ratio
        self._unload_power = 100

    def _set(self, power:int):
        """ Sets the intake motor's power.
        power: will be truncated to a value between [0,100].
               Intake is positive, unload is negative. 
               Motor will not turn for abs(power) <~ 5, due to friction."""
        # The segmented mapping function to 
        # map input[-100,100] to 'servo' angle [0,36]U{90}U[144,180]
        if power < -100:
            power = 0
        elif power < -5:
            power = int(0.379*power + 37.895)
        elif power <= 5:
            power = 90
        elif power < 100:
            power = int(0.379*power + 142.105)
        else:
            power = 180
        bot.set_pwm_servo(self._port, power)
        print(f'Intake: port {self._port}, eqvl servo angle {power}')

    def set_eat_power(self, power:int):
        """ Sets the intaking power """
        self._eat_power = abs(power)

    def set_unload_power(self, power:int):
        """ Sets the unloading power """
        self._unload_power = abs(power)

    def eat(self, power:int=None):
        """ Intake the table tennis balls. 
        power: optional, sets intaking power for the current action.
        To change intaking power permanently, use set_eat_power() """
        power = self._eat_power if power is None else abs(power)
        self._set(power)

    def unload(self, power:int=None):
        """ Unload the table tennis balls. 
        power: optional, sets unloading power for the current action.
        To change unloading power permanently, use set_unload_power() """
        power = self._unload_power if power is None else abs(power)
        self._set(-power)

    def set_free_drive(self):
        """Sets the motor to free drive mode."""
        self._set(0)

    def test_run(self):
        """ Tests basic intaking functions """
        intake = Intake()
        intake.eat()
        intake.unload(50)
        time.sleep(2)
        intake.set_free_drive()


class Buzzer:
    """ Class to represent the buzzer on the Ros board. """

    def __init__(self):
        pass


    def beep_pattern(self, pattern:str='...', repeat:int=1):
        beep_time = 250
        interval = 20
        if not all(char in ('.', '-', ' ') for char in pattern):
            # Indicate the pattern string is in an incorrect format
            for _ in range(10):
                bot.set_beep(10)
                time.sleep(5)
        else:
            for _ in range(repeat):
                for char in pattern:
                    if char == '.':
                        bot.set_beep(beep_time)
                        time.sleep((beep_time + interval)*0.001)
                    elif char == '-':
                        bot.set_beep(beep_time * 2)
                        time.sleep((beep_time * 2 + interval)*0.001)
                    elif char == ' ':
                        time.sleep((beep_time + interval)*0.001)


class Chassis:
    def __init__(self) -> None:
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.yaw_tolerance = 1
        self.yaw_rate = 0
        self.time_global_start = time.time()
        self.imu_global_start = bot.get_imu_attitude_data()[2]
        self.turn_num = 0
        self.intake = Intake()
        self.ultrasound = Ultrasound()
        self.event_handler = Event_Handler()
    
    def measure_stationary_yaw_drift_rate(self, duration, plot=False):
        yaws=[]
        times=[]

        start = time.time()
        roll, pitch, yaw = bot.get_imu_attitude_data()
        end = time.time()
        
        while end - start < duration:
            end = time.time()
            roll_end, pitch_end, yaw_end = bot.get_imu_attitude_data()
            if plot:
                times.append(end-start)
                yaws.append(yaw_end)

        if plot:
            plt.plot(times,yaws)
            plt.savefig('plt_no_offset.png')
        
        yaw_rate = (yaw_end - yaw)/(end-start)

        self.yaw_rate = yaw_rate

        return yaw_rate

    # Not needed
    def get_imu_after_calibration(self, yaw_rate, duration, plot=False):

        yaws=[]
        times=[]

        start = time.time()
        roll, pitch, yaw = bot.get_imu_attitude_data()
        yaw = yaw - (time.time() - self.time_global_start)*yaw_rate
        end = time.time()
        
        while end - start < duration:
            end = time.time()
            roll_end, pitch_end, yaw_end = bot.get_imu_attitude_data()
            yaw_end = yaw_end - (time.time() - self.time_global_start)*yaw_rate

            if plot:
                times.append(end-start)
                yaws.append(yaw_end)
        if plot:
            plt.plot(times,yaws)
            plt.savefig('plt_with_offset.png')
    
    def get_yaw_calibrated(self):
        t = time.time()
        _, _, yaw = bot.get_imu_attitude_data()
        # print('Yaw Before Calibration: {}'.format(yaw))
        yaw = yaw - (t - self.time_global_start)*self.yaw_rate
        # print('Yaw After Calibration: {}'.format(yaw))
        return yaw

    def action(self, controller, yaw_start):
        self.event_handler.check_restart()
        yaw = self.get_yaw_calibrated()
        control = controller(yaw)
        error = yaw_start - yaw 
        self.vz = max(-10, min(control, 10))
        print("Error: {}, Control: {}, Vz: {}".format(error, control, self.vz))
        bot.set_car_motion(self.vx, self.vy, self.vz)
        # self.intake.set_eat_power()
        time.sleep(0.05)

    def forward(self, duration = None):
        print('Chassis Foward Checkpoint 1')
        yaw_start = self.get_yaw_calibrated()
        pid_forward = PID(0.5,0,0.1, setpoint=yaw_start)
        self.vx = 0.2
        self.vy = 0
        if duration == None:
            while not(self.ultrasound.object_front):
                try:
                    self.ultrasound.receive_distances()
                    distances = self.ultrasound.get_distances()  
                    print("Distances:", distances)
                    print("Obstacles at directions: {}".format(self.ultrasound.check_obstacle))

                    self.action(pid_forward, yaw_start)

                except KeyboardInterrupt:
                    self.stop()
                    break
        else:
            start = time.time()
            end = time.time()
            while (end - start < duration) and not(self.ultrasound.object_front):
                end = time.time()
                self.ultrasound.receive_distances()
                distances = self.ultrasound.get_distances()  
                print("Distances:", distances)
                print("Obstacles at directions: {}".format(self.ultrasound.check_obstacle))
                self.action(pid_forward, yaw_start)
        self.stop()
               
    def backward(self, duration = None):
        yaw_start = self.get_yaw_calibrated()
        pid_backward = PID(0.5,0,0.1, setpoint=yaw_start)
        self.vx = -0.2
        self.vy = 0
        if duration == None:
            while not(self.ultrasound.object_back):
                try:
                    self.ultrasound.receive_distances()
                    distances = self.ultrasound.get_distances()  
                    print("Distances:", distances)
                    print("Obstacles at directions: {}".format(self.ultrasound.check_obstacle))
                    self.action(pid_backward, yaw_start)
                except KeyboardInterrupt:
                    self.stop()
                    break
        else:
            start = time.time()
            end = time.time()
            while (end - start < duration) and not(self.ultrasound.object_back):
                end = time.time()
                self.ultrasound.receive_distances()
                distances = self.ultrasound.get_distances()  
                print("Distances:", distances)
                print("Obstacles at directions: {}".format(self.ultrasound.check_obstacle))
                self.action(pid_backward, yaw_start)
        self.stop()
    
    def right(self, duration = None):
        yaw_start = self.get_yaw_calibrated()
        pid_right = PID(0.2, -0.5, 0.01, setpoint=yaw_start)
        self.vx = 0
        self.vy = 0.2
        if duration == None:
            while not(self.ultrasound.object_right):
                try:
                    self.ultrasound.receive_distances()
                    self.action(pid_right, yaw_start)
                except KeyboardInterrupt:
                    self.stop()
                    break
        else:
            start = time.time()
            end = time.time()
            while (end - start < duration) and not(self.ultrasound.object_right):
                self.ultrasound.receive_distances()
                end = time.time()
                self.action(pid_right, yaw_start)
        self.stop()
        
    def left(self, duration = None):
        yaw_start = self.get_yaw_calibrated()
        pid_left = PID(0.2, -0.5, 0.01, setpoint=yaw_start)
        self.vx = 0
        self.vy = -0.2
        if duration == None:
            while not(self.ultrasound.object_left):
                try:
                    self.ultrasound.receive_distances()
                    self.action(pid_left, yaw_start)
                except KeyboardInterrupt:
                    self.stop()
                    break
        else:
            start = time.time()
            end = time.time()
            while (end - start < duration) and not(self.ultrasound.object_left):
                self.ultrasound.receive_distances()
                end = time.time()
                self.action(pid_left, yaw_start)
        self.stop()
    
    def turn(self, angle): # -ve for right, +ve for left
        yaw_start = self.get_yaw_calibrated()
        pid_turn = PID(0.07, 0, 0.03, setpoint = yaw_start + angle)
        self.vx = 0
        self.vy = 0
        yaw = yaw_start
        previous_yaw = yaw_start
        while abs(yaw_start + angle - yaw) > 1:
            try:
                yaw = self.get_yaw_calibrated()
                if (yaw-previous_yaw) > 300:
                    yaw -= 360
                elif (yaw-previous_yaw) < -300:
                    yaw += 360
                previous_yaw = yaw
                error = yaw_start + angle - yaw 
                control = pid_turn(yaw)
                self.vz = max(-10, min(control, 10))
                print("Error: {}, Control: {}, Vz: {}".format(error, control, self.vz))
                bot.set_car_motion(self.vx, self.vy, self.vz)
                # self.intake.set_eat_power()
                time.sleep(0.1)
            except KeyboardInterrupt:
                self.stop()
                break

        ######
        if (angle < 0): #Assuming turns can only be +90 or -90
            self.turn_num -= 1  
        else:
            self.turn_num += 1

        self.stop()    
    
    def stop(self):
        bot.set_car_motion(0, 0 ,0)
        # self.intake.set_unload_power()

    # May not need
    def find_base(self):
        # Input: 4 sets of xy coordinates - points
        # Might have to make Lidar an attribute of Chassis Class, and call methods of Lidar
        points = []
        if self.turn % 4 == 0: # Base in Quadrant 3
            for point in points:
                if point[0] < 0 and point[1] < 0:
                    return point
        elif self.turn % 4 == 1: # Base in Quadrant 2
            for point in points:
                if point[0] < 0 and point[1] > 0:
                    return point
        elif self.turn % 4 == 2: # Base in Quadrant 1
            for point in points:
                if point[0] > 0 and point[1] > 0:
                    return point
        elif self.turn % 4 == 3:
            for point in points: # Base in Quadrant 4
                if point[0] > 0 and point[1] < 0:
                    return point
    
    def revert_orientation(self):
        num = self.turn_num - 2
        print(num)
        if num%4 == 3:
            self.turn(90)
            time.sleep(0.5)
        else:
            for i in range((num-2)%4):
                self.turn(-90)
                time.sleep(0.5)

    


pass
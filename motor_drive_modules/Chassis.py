import Constants
from PIDController import PIDController
import time
from RosmasterBoard import Rosmaster
import threading

# global array to store motor velocity
# N.B. Actual velocity of the wheel is this times a constant
motor_velocity_array = [0,0,0,0]

bot = Rosmaster()
bot.create_receive_threading()
# Enable auto data sending, and meant to be temporary (not forever)
bot.set_auto_report_state(enable = True, forever = False)

class Motor:
    def __init__(self, port):
        """ Initializes the motor on the given port. """
        self.port = port    # port of the motor
        self.output_power = 0   # output pwr, determines pwm ratio
        self.position = 0   # position of encoder
        self.tolerance = 10
        # PID parameters
        self.kP = 1
        self.kI = 0
        self.kD = 0
        self.position = self.update_position()

    def set(self, power, reverse=False):
        """ Sets the motor's drive power, with an option to reverse direction. """
        """ Power range from -100 to 100 """
        self.output_power = -power if reverse else power
        motor_velocity_array[self.port] = self.output_power
        bot.set_motor(motor_velocity_array[0], motor_velocity_array[1], motor_velocity_array[2], motor_velocity_array[3])

    def update_position(self):
        """ Updates the motor's position (acquiring from bot) """
        encoder_readings = bot.get_motor_encoder()
        position = encoder_readings[self.port]
        self.position = position
        time.sleep(0.001)
        return self.position

    def set_free_drive(self):
        """Sets the motor to free drive mode."""
        self.set(0)

    def set_pid_coefficient(self, kP, kI, kD):
        self.kP = kP
        self.kI = kI
        self.kD = kD

    def set_position(self, target_position):
        """Moves the motor to a specific position using PID control."""
        self.update_position()
        position_pid = PIDController(self.kP, self.kI, self.kD)
        while abs(target_position - self.position) >= self.tolerance:
            self.update_position()  # Update positional readings of the motor
            pid_output = position_pid.calculate(self.position, target_position)
            # Ensure control effort is within [-100, 100] range
            control_effort = max(min(pid_output, 100), -100)
            self.set(control_effort)
            print(">>>>>>", target_position - self.position)
            time.sleep(0.001)
        self.set(0)  # Stop the motor once target position is within tolerance

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


class Intake:
    def __init__(self, positive_port=2, negative_port=3):
        """ Initializes the intake motor on the given port. """
        self._positive_port = positive_port    # positive port of the motor
        self._negative_port = negative_port    # negative port of the motor
        self._eat_power = 50   # intaking pwr, determines pwm ratio
        self._unload_power = 50

    def _set(self, power:int):
        """ Sets the intake motor's power.
        power: will be truncated to a value between [0,100].
               Intake is positive, unload is negative. """
        # record the sign of power
        eat = 1 if power >= 0 else 0
        power = abs(power)
        # Truncate power to [0,100]
        power = max(0, min(100, power))
        # Map power from [0,100] to [0, 180] to mimmick a servo
        power = int(power / 100 * 180)
        if eat:
            bot.set_pwm_servo(self._negative_port, 0)
            bot.set_pwm_servo(self._positive_port, power)
        else:
            bot.set_pwm_servo(self._positive_port, 0)
            bot.set_pwm_servo(self._negative_port, -power)

    def set_free_drive(self):
        """Sets the motor to free drive mode."""
        self._set(0)

    def set_eat_power(self, power):
        """ Sets the intaking power """
        self._eat_power = power

    def eat(self, power=None):
        """ Intake the table tennis balls. """
        if not power is None:
            self._eat_power = power
        bot.set_pwm_servo(self._port, self._eat_power)

    def unload(self, power=None):
        """ Intake the table tennis balls. """
        if not power is None:
            self._unload_power = power
        bot.set_pwm_servo(self._port, -self._unload_power)


class MecanumDrive:
    def __init__(self):
        """Initializes the mecanum drive with four motors."""
        self.front_left_wheel = Motor(Constants.DriveConst.front_left_port)
        self.front_right_wheel = Motor(Constants.DriveConst.front_right_port)
        self.back_left_wheel = Motor(Constants.DriveConst.back_left_port)
        self.back_right_wheel = Motor(Constants.DriveConst.back_right_port)
        self.wheel_speeds = {'front_left': 0, 'front_right': 0, 'back_left': 0, 'back_right': 0}
        self.target_wheel_speeds = {'front_left': 0, 'front_right': 0, 'back_left': 0, 'back_right': 0}

        """X. Y (Traditional Cartesian Definition) and Rot (Clockwise +ve)"""
        self.position = [0, 0, 0]
        self.target_position = [0, 0, 0]

        """Initializes the tolerance for linear and rotational position"""
        self.lin_tolerance = 0.1
        self.rot_tolerance = 1

    def calculate_mecanum_drive_speeds(self, forward_speed, strafe_speed, rotate_speed):
        """ Calculates wheel speeds for the mecanum drive system. """
        calculated_wheel_speeds = {'front_left': forward_speed + strafe_speed + rotate_speed,
                                  'front_right': forward_speed - strafe_speed - rotate_speed,
                                  'back_left': forward_speed - strafe_speed + rotate_speed,
                                  'back_right': forward_speed + strafe_speed - rotate_speed}
        max_speed = max(abs(speed) for speed in calculated_wheel_speeds.values())
        if max_speed > 1:
            for key in calculated_wheel_speeds:
                calculated_wheel_speeds[key] /= max_speed
        return calculated_wheel_speeds

    def update_motor_positions(self):
        """ Updates the positions of all motors. """
        for motor in [self.front_left_wheel, self.front_right_wheel, self.back_left_wheel, self.back_right_wheel]:
            motor.update_position()

    def set_chassis_position(self, target_chassis_position):
        """Sets the chassis to a target position using PID control."""
        self.target_position = target_chassis_position
        x_pid = PIDController(1, 0, 0)
        y_pid = PIDController(1, 0, 0)
        rot_pid = PIDController(1, 0, 0)
        while not all(abs(self.target_position[i] - self.position[i]) < self.lin_tolerance if i < 2 else abs(self.target_position[i] - self.position[i]) < self.rot_tolerance for i in range(3)):
            self.update_motor_positions()  # Update positions of each motor
            x_control = x_pid.calculate(self.position[0], self.target_position[0])
            y_control = y_pid.calculate(self.position[1], self.target_position[1])
            rot_control = rot_pid.calculate(self.position[2], self.target_position[2])
            calculated_speeds = self.calculate_mecanum_drive_speeds(y_control, x_control, rot_control)
            self.set_target_wheel_speeds(calculated_speeds)

    def set_target_wheel_speeds(self, target_speeds):
        """Sets the target speeds for each wheel."""
        self.target_wheel_speeds = target_speeds
        for wheel, speed in self.target_wheel_speeds.items():
            getattr(self, wheel).set(speed)

    def get_wheel_speeds(self):
        """Returns the current speeds of the wheels."""
        return self.wheel_speeds

    def set_current_chassis_position(self, set_position):
        """Sets the current position of the robot."""
        self.position = set_position

    def get_current_chassis_position(self):
        """Returns the current position of the robot. (Need readings from IMU through Serial)"""
        return self.position

motor_0 = Motor(0)
motor_1 = Motor(1)
motor_2 = Motor(2)
motor_3 = Motor(3)

motor_0.set_pid_coefficient(0.8, 5, 6)
motor_1.set_pid_coefficient(0.8, 5, 6)
motor_2.set_pid_coefficient(0.8, 5, 6)
motor_3.set_pid_coefficient(0.8, 5, 6)

threads = []
motors = [motor_0, motor_1, motor_2, motor_3]

def set_motor_positions(positions):
    threads = []
    for motor, position in zip(motors, positions):
        thread = threading.Thread(target=motor.set_position, args=(position,))
        thread.append(thread)
        thread.start()
    for thread in threads:
        thread.join()

set_motor_positions([5000, 5000, 5000, 5000])
set_motor_positions([0, 0, 0, 0])

# def test_servo():
#     """ test basic servo functions """
#     servo = Servo(1)
#     print(servo.get_position())
#     while 1:
#         servo.grip()
#         time.sleep(1)
#         servo.release()
#         time.sleep(1)

def test_intake():
    intake = Intake()
    intake._set(0)
    input()
    intake._set(50)
    time.sleep(1)
    intake._set(-100)
    time.sleep(1)

# test_intake()

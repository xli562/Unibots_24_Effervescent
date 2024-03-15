import Constants
import PIDController
import time
from RosmasterBoard import Rosmaster

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
        self.tolerance = 1
        # PID parameters
        self.kP = 1
        self.kI = 0
        self.kD = 0

    def set(self, power, reverse=False):
        """ Sets the motor's drive power, with an option to reverse direction. """
        """ Power range from -100 to 100 """
        self.output_power = -power if reverse else power
        print(motor_velocity_array, self.port)
        motor_velocity_array[self.port] = self.output_power
        bot.set_motor(motor_velocity_array[0], motor_velocity_array[1], motor_velocity_array[2], motor_velocity_array[3])

    def update_position(self, position):
        """ Updates the motor's position (acquiring from bot) """
        encoder_readings = bot.get_motor_encoder()
        position = encoder_readings[self.port]
        self.position = position

    def set_free_drive(self):
        """Sets the motor to free drive mode."""
        self.set(0)

    def set_pid_coefficient(self, kP, kI, kD):
        self.kP = kP
        self.kI = kI
        self.kD = kD

    def set_position(self, target_position):
        """ Moves the motor to a specific position using PID control. """
        position_pid = PIDController(self.kP, self.kI, self.kD)
        while not(abs(target_position - self.position) < self.tolerance):
            self.update_position()  # Update positional readings of the motor
            self.set(position_pid.calculate(self.position, target_position))

    def set_brake(self):
        """ Sets the motor to brake mode. """
        self.set_position(self.position)


class Servo():
    """ The class to represent the gripper's servo. """
    
    def __init__(self, port):
        self._port = port
        self._position = 0   # target angular position of servo
    
    def set_position(self, degrees):
        """ Rotate the servo to specified angular position. """
        # Ensure degrees is within 0 to 180
        degrees = max(0, min(180, degrees))
        bot.set_pwm_servo(self._port, degrees)
        self._position = degrees

    def get_position(self) -> int:
        return self._position

    def grip(self):
        """ Grip a mini rugby. """
        ''' TODO: Calibration '''
        pass

    def release(self):
        """ Release a mini rugby. """
        ''' TODO: Calibration '''
        pass


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

def test_motor():
    """ Test basic motor functionalities """
    motor_test = Motor(0)
    motor_test.set_pid_coefficient(1, 0, 0)
    motor_test.update_position(0)
    motor_test.set(50)
    time.sleep(3)
    motor_test.set(0)
    motor_test.set(-50)
    time.sleep(3)
    motor_test.set(0)

def test_servo():
    """ test basic servo functions """
    servo = Servo(1)
    print(servo.get_position())

    servo.set_position(90)
    print(servo.get_position())
    time.sleep(2)
    print(servo.get_position())

    servo.set_position(0)
    print(servo.get_position())
    time.sleep(2)
    print(servo.get_position())

    servo.set_position(180)
    print(servo.get_position())
    time.sleep(2)
    print(servo.get_position())

    servo.set_position(90)
    print(servo.get_position())
    time.sleep(2)
    print(servo.get_position())

test_servo()

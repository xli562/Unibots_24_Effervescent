import Constants
import PIDController

class Motor:
    def __init__(self, port):
        """Initializes the motor on the given port."""
        self.port = port
        self.output_power = 0
        self.position = 0
        self.tolerance = 1

    def set(self, power, reverse=False):
        """Sets the motor's power, with an option to reverse direction."""
        self.output_power = -power if reverse else power

    def update_position(self):
        """Updates the motor's position (acquiring from Serial)"""
        return 0

    def set_free_drive(self):
        """Sets the motor to free drive mode."""
        self.set(0)

    def set_position(self, target_position):
        """Moves the motor to a specific position using PID control."""
        position_pid = PIDController(1, 0, 0)
        while not(abs(target_position - self.position) < self.tolerance):
            self.update_position()  # Update positional readings of the motor
            self.set(position_pid.calculate(self.position, target_position))

    def set_brake(self):
        """Sets the motor to brake mode."""
        self.set_position(self.position)


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
        """Calculates wheel speeds for the mecanum drive system."""
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
        """Updates the positions of all motors."""
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

    def set_current_position(self, set_position):
        """Sets the current position of the robot."""
        self.position = set_position

    def get_current_position(self):
        """Returns the current position of the robot. (Need readings from IMU through Serial)"""
        return self.position
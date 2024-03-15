import Constants

class PIDController:
    def __init__(self, kP, kI, kD):
        self.P = kP
        self.I = kI
        self.D = kD
        self.prev_error = 0
        self.integral = 0

    #Control effort calculation
    def calculate(self, current, target, dt=Constants.PID.dt):
        error = target - current
        derivative = (error - self.prev_error) / dt
        self.integral = self.integral + error * dt

        control_effort = (self.P * error) + (self.I * self.integral) + (self.D * derivative)

        self.prev_error = error
        return control_effort

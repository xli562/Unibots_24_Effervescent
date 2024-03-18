import Constants
from decimal import Decimal

class PIDController:
    def __init__(self, kP, kI, kD):
        self.P = kP
        self.I = kI
        self.D = kD
        self.prev_error = 0
        self.integral = 0

    def calculate(self, current, target, dt=Constants.PID.dt):
        """ Control effort calculation """
        error = target - current
        
        derivative = (error - self.prev_error) / Decimal(dt)
        self.integral = self.integral + Decimal(error) * Decimal(dt)

        control_effort = (self.P * Decimal(error)) + (self.I * self.integral) + (self.D * derivative)

        self.prev_error = Decimal(error)
        return control_effort

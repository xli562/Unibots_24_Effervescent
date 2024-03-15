from RosmasterBoard import Rosmaster

bot = Rosmaster()

class Servo():
    """ The class to represent the gripper's servo. """
    def __init__(self, port):
        self._port = port
    
    def set_position(self, degrees):
        """ Rotate the servo to specified angular position. """

    def grip(self):
        """ Grip a mini rugby. """
        ''' TODO: Calibration '''
        pass

    def release(self):
        """ Release a mini rugby. """
        ''' TODO: Calibration '''
        pass

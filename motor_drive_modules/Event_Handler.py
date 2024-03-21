import serial
import time

index_out = 0
index_in = 0

class Event_Handler():

    def __init__(self):
        self.ser = serial.Serial("/dev/ttyACM0", 115200)
        self.reset_flag = False
        self.timeout_flag = False
        
    
    def check_restart(self):
        """
        Check the Serial. If the message of "!Restart!" (sent from Arduino) is present, break from the main loop.
        """
        print('restart_point_1')
        try:
            if self.ser.in_waiting > 0:
                data_str = self.ser.readline().strip().decode('utf-8')
                print('Received Data: {}'.format(data_str))
                readings = data_str.split('!')
                for reading in readings:
                    if reading == "Restart":  # Command to restart - sent when restart button of Arduino is pressed
                        print('RESTARTING THE ROBOT')
                        self.reset_flag = True
                        break # CHECK IF NEED THIS
        except Exception as e:
            print(e)

    def empty_events(self):
        self.reset_flag = False


event_handler = Event_Handler()

while True:
    index_out += 1
    index_in = 0
    print('Out Index: {}'.format(index_out))
    while True:
        print('In Index: {}'.format(index_in))
        index_in += 1
        event_handler.check_restart()
        if event_handler.reset:
            event_handler.empty_events()
            break
        time.sleep(0.1)

        # Do the loop actions
        # Check for events - break





import serial
import time

index_out = 0
index_in = 0
ser = serial.Serial("/dev/ttyACM0", 115200)

class Event_Handler():

    def __init__(self):
        self.reset = False
    
    def check_restart(self):
        print('restart_point_1')
        try:
            if ser.in_waiting > 0:
                data_str = ser.readline().strip().decode('utf-8')
                print('Received Data: {}'.format(data_str))
                readings = data_str.split('!')
                for reading in readings:
                    if reading == "Restart":  # Command to restart - sent when restart button of Arduino is pressed
                        print('RESTARTING THE ROBOT')
                        self.reset = True
        except Exception as e:
            print(e)

    def empty_events(self):
        self.reset = False

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





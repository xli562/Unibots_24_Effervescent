import serial
import time

index_out = 0
index_in = 0

class Event_Handler():

    def __init__(self, ser):
        self.ser = ser
        self.reset_flag = False
        self.timeout_flag = False
        self.timeout_duration = 30
        # Track the starting time of current main loop iteration. 
        # When (current time - this) > timeout_duration, should start returning.
        self.iteration_start_time = time.time()
        
    
    def check_reset(self):
        """ Check the Serial. If the message of "!Restart!" 
        (sent from Arduino) is present, break from the main loop. """
        
        # print('restart_point_1')
        try:
            if self.ser.in_waiting > 0:
                data_str = self.ser.readline().strip().decode('utf-8')
                # print('Received Data: {}'.format(data_str))
                readings = data_str.split('!')
                for reading in readings:
                    if reading == "Restart":  # Command to restart - sent when restart button of Arduino is pressed
                        print('RESTARTING THE ROBOT')
                        self.reset_flag = True
        except Exception as e:
            print(e)
    
    
    def check_timeout(self):
        current_time = time.time()
        if current_time - self.iteration_start_time > self.timeout_duration:
            self.timeout_flag = True
            # break
            # 目前还没有加入上面这行的break，后面可以考虑一下这里需不需要这个break


    def empty_events(self):
        self.reset_flag = False
        self.timeout_flag = False


# event_handler = Event_Handler()

# while True:
#     index_out += 1
#     index_in = 0
#     print('Out Index: {}'.format(index_out))
#     while True:
#         print('In Index: {}'.format(index_in))
#         index_in += 1
#         event_handler.check_restart()
#         if event_handler.reset_flag:
#             event_handler.empty_events()
#             break
#         time.sleep(0.1)

        # Do the loop actions
        # Check for events - break





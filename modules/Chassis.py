from .RosmasterBoard import Rosmaster

import time, subprocess, serial, threading, re
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from simple_pid import PID
import numpy as np


bot = Rosmaster()
bot.create_receive_threading()
# Enable auto data sending every 40ms.
bot.set_auto_report_state(enable = True)
# Clear cache sent from the Rosmaster board
bot.clear_auto_report_data()



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



class Servo:
    """ Class to represent the gripper's servo. """
    
    def __init__(self, port=1):
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
        :param power: will be truncated to a value between [0,100].
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
        # print(f'Intake: port {self._port}, eqvl servo angle {power}')


    def set_eat_power(self, power:int):
        """ Sets the intaking power """

        self._eat_power = abs(power)


    def set_unload_power(self, power:int):
        """ Sets the unloading power """

        self._unload_power = abs(power)


    def eat(self, power:int=None):
        """ Intake the table tennis balls. 
        :param power: optional, sets intaking power for the current action.
            To change intaking power permanently, use set_eat_power() """

        power = self._eat_power if power is None else abs(power)
        self._set(power)


    def unload(self, power:int=None):
        """ Unload the table tennis balls. 
        :param power: optional, sets unloading power for the current action.
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



class Lidar:
    """ Class to represent the lidar.
    Depends on class Buzzer (slightly) for error reporting. """

    _instance = None    # Needed for the __new__() method

    def __init__(self, beep_pattern):
        """ Initialise an instance of Lidar.
        :param beep_pattern: the beeping function from class Buzzer """

        self._blocking = False
        self._beep_pattern = beep_pattern
        # Threads management
        self._threads = []
        self._terminate_all_threads = threading.Event()
        self._terminate_all_threads.clear()
        self._last_reading = np.array([0., 0., 0.])
        # A list of fixed length 450 to store data for the last complete cycle
        self._last_cycle_readings = np.array([[0., 0., 0.]])
        # A list of growing length to store data for the current cycle
        self._current_cycle_readings = np.array([[0., 0., 0.]])
        self._last_fitted_angle_error = 0
        self._last_fitted_arena_vertices = np.array([[0., 0.], [0., 0.], [0., 0.], [0., 0.]])
        self._start_autoreceive_readings_thread()
        self._start_periodic_fitting_thread()
        self._check_connection(timeout=5)
        self._new_data_available = False
    

    # def __new__(cls, *args, **kwargs):
    #     """ To prevent the Lidar instance from being created
    #     multiple times. """

    #     if not cls._instance:
    #         cls._instance = super(Lidar, cls).__new__(cls, *args, **kwargs)
    #     return cls._instance


    def __del__(self):
        self._terminate_all_threads.set()
        for thread in self._threads:
            thread.join()
        self._terminate_all_threads.clear()


    def _check_connection(self, timeout=5):
        """ Checks the Lidar connection.
        If the last reading is still empty after 5 seconds, 
        buzz the buzzer in a specific pattern if buzzer is connected. 
        
        :param timeout: the wait time before checking if lidar returns readings"""

        def check_reading():
            while not self._terminate_all_threads.is_set():
                if self._last_reading.size == 0:
                    print(f'Warning: Not receiving data from Lidar after timeout={timeout} seconds')
                    try:
                        self._beep_pattern('....-.')
                    except:
                        pass
        thread = threading.Timer(timeout, check_reading)
        self._threads.append(thread)
        thread.start()


    def _start_autoreceive_readings_thread(self):
        """ Creates thread to listen to cpp stdout. """

        # The path to the directory where you want to run the command
        cpp_file_folder = './lidar_sdk/build'
        # The command you want to run
        cpp_file = './blocking_test' if self._blocking else './non-blocking_test'
        self.process = subprocess.Popen(cpp_file, stdout=subprocess.PIPE, 
                                stderr=subprocess.PIPE, bufsize=1, 
                                universal_newlines=True, 
                                cwd=cpp_file_folder, shell=True)
        # Thread function for capturing output
        def capture_output(pipe):
            while not self._terminate_all_threads.is_set():
                # Pattern for regex parsing
                pattern = re.compile(r'\[(\d+): ([\d.]+), ([\d.]+)\]')
                for line in iter(pipe.readline, ''):
                    match = pattern.search(line)
                    if match:
                        index = int(match.group(1))
                        r = float(match.group(2))  # Radius (distance)
                        theta = float(match.group(3))  # Angle
                        self._last_reading = np.array([index, r, theta])
                        self._current_cycle_readings = np.vstack((self._current_cycle_readings, self._last_reading))
                        if index == 0:
                            # Update fresh data into last-cycle-reading
                            self._last_cycle_readings = self._current_cycle_readings
                            # Reset current cycle for each new cycle
                            self._current_cycle_readings = np.array([[0., 0., 0.]])

        # Data queue and thread setup
        stdout_thread = threading.Thread(target=capture_output, 
                                         args=(self.process.stdout,))
        self._threads.append(stdout_thread)
        stdout_thread.start()


    def _start_periodic_fitting_thread(self, period=0.1):
        """ Creates thread to periodically fit the square 
        and store results in instance variables. """

        def get_fit(period):
            while not self._terminate_all_threads.is_set():
                fraction_of_period = period * 0.1
                start_time = time.time()
                self._last_fitted_angle_error, self._last_fitted_arena_vertices = self.fit_square(reduce_data_size_step=3)
                while time.time() < start_time + period:
                    time.sleep(fraction_of_period)
        thread = threading.Thread(target=get_fit, 
                                         args=(period,))
        self._threads.append(thread)
        thread.start()


    def get_last_cycle_readings(self) -> np.ndarray:
        """ This function reads from a cached variable
        and takes minimal processing effort 

        :return: an np.ndarray of the last cycle's readings.
            returns a 2D np.array([[0, 0., 0.]]) if no data is returned from the lidar. """

        if self._last_cycle_readings.size > 0:
            return self._last_cycle_readings
        else:
            return np.array([[0, 0., 0.]])
    

    def get_last_reading(self) -> np.ndarray:
        """ This function reads from a cached variable
        and takes minimal processing effort 

        :return: an array of [index, r, theta] of the last point being read. 
            returns a 1D np.array([0, 0., 0.]) if no data is returned from the lidar. """
        if self._last_reading.size  > 0:
            return self._last_reading
        else:
            return np.array([0, 0., 0.])


    def get_last_angle_error(self):
        """ Getter for last measured angle error, for use in navigation
        
        :return: The last measured angle error,
            read from a cached variable updated at ~10Hz. """
        
        return self._last_fitted_angle_error
    

    def get_last_arena_vertices(self):
        """ Getter for last measured arena vertices
        
        :return: The last fitted arena vertices, 
            read from a cached variable updated at ~10Hz. """
        
        return self._last_fitted_arena_vertices


    def _polar_to_cartesian(self, r: float, theta: float) -> np.ndarray:
        """ Convert polar coordinates to cartesian coordinates. 
        :param r: polar radius
        :param theta: polar angle *in degrees* """

        x = r * np.cos(np.radians(theta))
        y = r * np.sin(np.radians(theta))
        return np.array([x, y])


    def _find_bounding_sides(self, points:np.ndarray) -> np.ndarray:
        """ Find the bounding box to the given points. 

        :param points: 2D np.array([[x1, y1], [x2, y2], ..., [xn, yn]]) of
            cartesian point coord.s
        :returns: Min and max x and y values as below.
            np.array([min_x, min_y, max_x, max_y]) """
        
        if points.size > 0:
            min_x = np.min(points[:, 0])
            max_x = np.max(points[:, 0])
            min_y = np.min(points[:, 1])
            max_y = np.max(points[:, 1])
        else:
            min_x, min_y, max_x, max_y = (0., 0., 0., 0.)
        return np.array([min_x, max_x, min_y, max_y])


    def fit_square(self, reduce_data_size_step=2, 
                   angle_step=1) -> list:
        """ Finds the rotated minimum-area rectangle (MAR) of the point cloud.
         
        :param reduce_data_size_step: step for data size reduction
        :param angle_step: step for angle search, max is 10
        :return: rotation_angle, bounding_vertices
            angle_error: error in heading w.r.t. the arena wall 
                directly faced by the robot. 
            bounding_vertices: np.ndarray of 4 vertices of the min. area rect,
            see below for order.
            (max_x, max_y), (max_x, min_y), (min_x, min_y), (min_x, max_y) """
        
        # Foolproof inputs
        angle_step = max(0, min(angle_step, 10))

        # Preprocess the data
        # TODO: filter out very-far points
        i = 0
        cleaned_data = np.array([[0., 0.]])   # The data points to be considered
        for _, r, theta in self.get_last_cycle_readings():
            # Reduce data set size for faster computation
            if i % reduce_data_size_step == 0:
                i += 1
                continue
            cleaned_data = np.vstack((cleaned_data, self._polar_to_cartesian(r, theta)))
            i += 1
        if cleaned_data.size == 0:
            cleaned_data = np.array([[0., 0.]])
        # Find the first MAR and its center's coord.s
        bounding_sides = self._find_bounding_sides(cleaned_data)
        center_offset = np.array([0.5*(bounding_sides[0]+bounding_sides[1]),
                                  0.5*(bounding_sides[2]+bounding_sides[3])])
        # Center the cleaned data on the origin
        bounding_sides -= np.array([center_offset[0], center_offset[0],
                                   center_offset[1], center_offset[1]])
        cleaned_data -= center_offset

        def calc_MAR_area(bounds:np.ndarray) -> float:
            """ Calculates area of the MAR based on its bounds

            :param bounds: bounds of the MAR, as below.
            np.array([min_x, max_x, min_y, max_y]) """

            return (bounds[1]-bounds[0])*(bounds[3]-bounds[2])

        rotation_angles = np.array([0])
        bounding_sides_array = np.array(bounding_sides)
        areas = np.array([calc_MAR_area(bounding_sides)])  # List to hold areas of the min. area rect.s
        rotation_matrix = np.array([[1,0],  # rotation matrix to be applied
                                    [0,1]]) # to the points in the set
        # Rotate the point set and find minimum area
        i = 0

        def find_rot_ang_bounding_vertices(min_area_index:int):
            """ Calculates error in angle and bounding vertices from 
            argmin of the areas array
             
            :param min_area_index: The index at which the 
                areas array takes its minimum 
            :returns: angle_error, bounding_vertices
                angle_error: error in heading w.r.t. the arena wall 
                    directly faced by the robot. 
                bounding_vertices: np.ndarray of 4 vertices of 
                    the min. area rect, see below for order.
                    (max_x, max_y), (max_x, min_y), (min_x, min_y), (min_x, max_y) """
            
            rotation_angle = angle_step * min_area_index
            bounding_sides = bounding_sides_array[min_area_index]
            bounding_vertices = np.array([[bounding_sides[1], bounding_sides[3]],
                                        [bounding_sides[1], bounding_sides[2]],
                                        [bounding_sides[0], bounding_sides[2]],
                                        [bounding_sides[0], bounding_sides[3]]])
            inv_rotation_matrix = np.array([[np.cos(np.radians(rotation_angle)), np.sin(np.radians(rotation_angle))],
                                [-np.sin(np.radians(rotation_angle)), np.cos(np.radians(rotation_angle))]])
            bounding_vertices = (bounding_vertices @ inv_rotation_matrix.T) + center_offset
            # Map rotation angle to angle error for navigation
            angle_error = 0
            if rotation_angle < 45:
                angle_error = -rotation_angle
            elif rotation_angle < 90:
                angle_error = 90 - rotation_angle
            return angle_error, bounding_vertices
        
        for rotation_angle in range(angle_step, 91, angle_step):
            np.vstack((rotation_angles, rotation_angle))
            rotation_matrix = np.array([[np.cos(np.radians(rotation_angle)), -np.sin(np.radians(rotation_angle))],
                                       [np.sin(np.radians(rotation_angle)), np.cos(np.radians(rotation_angle))]])
            # Apply rotation to every point            
            rotated_data = cleaned_data @ rotation_matrix.T
            bounding_sides = self._find_bounding_sides(rotated_data)
            bounding_sides_array = np.vstack((bounding_sides_array, bounding_sides))
            areas = np.vstack((areas, calc_MAR_area(bounding_sides)))
            i += 1
        # calculate the argmin if it is not already found in the above loop
        i = np.argmin(areas)
        return find_rot_ang_bounding_vertices(i)


    def plot_fit_square(self, axes_range=None):
        """ Plots the point cloud and the fitted square from the lidar
        :param fit_square: a function
        :param get_last_cycle_readings: a function """
        def update_plot(frame, axes_range):
            plt.cla()  # Clear the current axes
            _, square_vertices = self.fit_square()
            print(f'square_vertices {square_vertices}')
            cycle_readings = self.get_last_cycle_readings()
            # Add the first point at the end to close the square
            square_vertices = np.vstack((square_vertices, square_vertices[0]))
            # This unpacks the vertices into x and y coordinates
            square_x, square_y = zip(*square_vertices)
            plt.plot(square_x, square_y, 'r-')  # Plot the square in red

            # Plot the points from the last cycle readings, with improved error handling
            if cycle_readings.size > 0:
                # Handle error caused by abnormal cycle_readings data
                try:
                    _, readings_r, readings_theta = zip(*cycle_readings)
                    readings_r = np.array(readings_r)
                    readings_theta = np.array(readings_theta)
                    readings_x = readings_r * np.cos(np.radians(readings_theta))
                    readings_y = readings_r * np.sin(np.radians(readings_theta))
                    # Auto determine the axes range
                    if axes_range is None:
                        max_x_y = max(np.max(np.abs(readings_x)), np.max(np.abs(readings_y)))
                        axes_range = [-max_x_y*1.5, max_x_y*1.5]
                    plt.scatter(readings_x, readings_y, s=10, color='blue')
                except ValueError as e:
                    print(f'cycle_readings = {cycle_readings}, \nError: {e}')

            plt.xlim(axes_range[0], axes_range[1])
            plt.ylim(axes_range[0], axes_range[1])

        # Setup the figure and axis
        plt.figure(figsize=(8, 8))

        # Create an animation that updates the plot
        ani = FuncAnimation(plt.gcf(), update_plot, interval=1000, frames=20, blit=False, fargs=(axes_range,))  # Update every 1000 ms

        # ani.save('myanimation.mp4', writer='ffmpeg')
        plt.show()



class Arduino:
    """ Class to represent the arduino. Reads ultrasound and reset_button datas. 
    Depends on class Buzzer (slightly) for error reporting. """

    _instance = None    # Needed for the __new__() method

    def __init__(self, beep_pattern):
        """ Initialises an instance of class Arduino
        :param beep_pattern: the beeping function from class Buzzer """
        # Serial port
        self._ser = serial.Serial("/dev/Arduino", 115200)
        self._beep_pattern = beep_pattern
        # Cached accessible readings
        self._last_ultrasound_readings = np.array([0., 0., 0., 0., 0.])
        self.ultrasound_new_reading_available = threading.Event()
        self.ultrasound_new_reading_available.clear()
        self.received_reset = threading.Event()
        self.received_reset.clear()
        # Threads management
        self._threads = []
        self._terminate_all_threads = threading.Event()
        self._terminate_all_threads.clear()
        self._start_scan_serial_thread()
        

    # def __new__(cls, *args, **kwargs):
    #     """ To prevent the Arduino instance from being created
    #     multiple times. """

    #     if not cls._instance:
    #         cls._instance = super(Arduino, cls).__new__(cls, *args, **kwargs)
    #     return cls._instance


    def __del__(self):
        self._terminate_all_threads.set()
        for thread in self._threads:
            thread.join()
        self._terminate_all_threads.clear()


    def _check_connection(self, timeout=5):
        """ Checks the Arduino connection.
        If the last reading is still empty after 5 seconds, 
        buzz the buzzer in a specific pattern if buzzer is connected. 
        
        :param timeout: the wait time before checking if arduino returns readings"""

        def check_arduino_reading():
            while not self._terminate_all_threads.is_set():
                if self._last_ultrasound_readings.size == 0:
                    print(f'Warning: Not receiving ultrasound readings from Arduino after timeout={timeout} seconds')
                    try:
                        self.buzzer.beep_pattern('.--..')
                    except:
                        pass
        thread = threading.Timer(timeout, check_arduino_reading)
        self._threads.append(thread)
        thread.start()


    def self_test(self):
        """ Tests the arduino class by 
        printing out (once) data received from the arduino. """

        print(f'_last_ultrasound_readings {self._last_ultrasound_readings}')
        print(f'_flag_received_reset {self.received_reset}')


    def _start_scan_serial_thread(self):
        """ Creates thread to scan Arduino's serial input """

        def listen_serial():
            while not self._terminate_all_threads.is_set():
                raw_line = self._ser.readline().strip()
                try:
                    line = raw_line.decode('utf-8')
                    if line.startswith("U"):
                        # Recieved ultrasonic reading
                        line = line[1:-2]
                        readings_str = line.split(',')
                        j = 0
                        for reading_str in readings_str:
                            if reading_str == '':
                                readings_str[j] = '255'
                            j += 1
                        self._last_ultrasound_readings = np.array([int(reading) for reading in readings_str])
                        self.ultrasound_new_reading_available.set()
                    elif line.startswith("R"):
                        self.received_reset.set()
                except Exception as e:
                    print(f'Exception while listening serial from Arduino: \n{e}')
        thread = threading.Thread(target=listen_serial)
        self._threads.append(thread)
        thread.start()


    def get_last_ultrasound_readings(self) -> np.ndarray:
        """ Returns last ultrasound readings read from arduino via serial """
        self.ultrasound_new_reading_available.clear()
        return self._last_ultrasound_readings



class Ultrasound:
    """ Class that reads ultrasound sensors from Arduino and identifies obstacles.
    Depends on class Arduino. """

    _instance = None    # Needed for the __new__() method

    def __init__(self, new_reading_available_event, 
                 get_last_ultrasound_readings, threashold_distance=20, 
                 validation_count=3):
        """ 
        :param new_reading_available_event: The event that indicates new 
            reading is available from class Arduino
        :param get_last_ultrasound_readings: Function to get the last 
            ultrasound readings from class Arduino
        :param threashold_distance: threashold for obstacle detection in cm 
        :param validation_count: how many times the threashold is met before
            we assume there is an obstacle. To avoid reading fluxuations 
            of the sensor. """

        self._detection_threashold = threashold_distance
        self._triggered_1 = np.array([0,0,0,0])
        self._validation_count = validation_count
        self._get_last_ultrasound_readings = get_last_ultrasound_readings
        self._new_reading_available_event = new_reading_available_event
        # Cached accessible readings
        # Right_Bottom, Left, Front, Back, Right_Top
        self._triggered = np.array([0, 0, 0, 0])
        # Robust triggering
        # 代表了连续几次iteration检测到了obstacle，为了避免随机触发
        self._detection_count = np.array([0, 0, 0, 0])
        self.rugby_trigger_count = 0 # 同理，代表连续几次检测到了rugby
        # Threads management
        self._threads = []
        self._terminate_all_threads = threading.Event()
        self._terminate_all_threads.clear()
        self._start_read_distances_thread()
        

    # def __new__(cls, *args, **kwargs):
    #     """ To prevent the Ultrasound instance from being created
    #     multiple times. """

    #     if not cls._instance:
    #         cls._instance = super(Ultrasound, cls).__new__(cls, *args, **kwargs)
    #     return cls._instance


    def __del__(self):
        self._terminate_all_threads.set()
        for thread in self._threads:
            thread.join()
        self._terminate_all_threads.clear()
    

    def _start_read_distances_thread(self):
        """ Creates thread to read ultrasound distances read from class Arduino """

        def read_distances():
            while not self._terminate_all_threads.is_set():
                # Wait for a new reading to become available
                # print('waiting...')
                time.sleep(0.1)
                distances:np.ndarray = self._get_last_ultrasound_readings()
                # print(f'distances{distances}')
                i = 0
                for distance in distances[:-1]: # [:-1] is to neglect the rugby detector
                    if distance <= self._detection_threashold and distance > 0:
                        # Increment detection_count for the direction 
                        # if reading is within threashold.
                        self._detection_count[i] += 1
                    else:
                        # Clear detection_count for the direction 
                        # if reading is larger than threashold.
                        self._detection_count[i] = 0
                    i += 1
                # FIXME: Not sure if sensitive to float division errors
                self._triggered = np.floor(self._detection_count / self._validation_count)
                j = 0
                
                for value in self._triggered:
                    self._triggered_1[j] = 0 if value == 0 else 1
                    j += 1
        thread = threading.Thread(target=read_distances)
        self._threads.append(thread)
        thread.start()
            

    def detect_rugby(self):
        # TODO: under construction
        # Update Rugby连续几次iteration被检测到了            
        if (self._distances[4] - self._distances[0]) >= 10:
            self.rugby_trigger_count += 1
        else:
            self.rugby_trigger_count = 0


    def self_test(self):
        """ Prints ultrasound readings once for testing """
        print(self._get_last_ultrasound_readings())


    def rugby_right(self):
        if (self.updated):
            return (self.rugby_trigger_count >= 5)
            # return (self.distances[4] - self.distances[0]) >= 5
            # rugby_left is True when the difference between top and bottom sensor > 5cm


    def check_obstacle(self, direction:str) -> bool:
        """ Determines whether the robot should start.

        :param direction: moving direction the robot
        :return: boolean value depending on if there is 
            an obstacle in the moving direction """
        
        if direction == 'f':
            return self._triggered[0]
        elif direction == 'b':
            return self._triggered[1]
        elif direction == 'r':
            return self._triggered[3]
        elif direction == 'l':
            return self._triggered[2]
        else:
            return self._triggered[0]
        

    def check_all_obstacles(self) -> np.ndarray:
        return self._triggered_1



class EventHandler:
    """ Class to set event flags to break out loops.
    Depends on class Arduino. """

    def __init__(self, received_reset:threading.Event, timeout_duration=30):
        """ 
        :param received_reset: event that indicates the arduino board is reset, 
            from class Arduino """
        
        self._received_reset = received_reset
        self._timeout_duration = timeout_duration
        # Flags
        self.reset_flag = False
        self.timeout_flag = False
        # Track the starting time of current main loop iteration. 
        # When (current time - this) > timeout_duration, should start returning.
        self.iteration_start_time = time.time()
        
    
    def check_reset(self):
        """ Check the Serial. If received "R" 
        (sent from Arduino) is present, break from the main loop. """
        
        # print('restart_point_1')
        if self._received_reset.is_set():
            print('RESTARTING THE ROBOT')
            self.reset_flag = True
    
    
    def check_timeout(self):
        """ Check if the program has been running longer than the 
        specified duration (self.timeout_duration) """

        current_time = time.time()
        if current_time - self.iteration_start_time > self._timeout_duration:
            self.timeout_flag = True
            # break
            # 目前还没有加入上面这行的break，后面可以考虑一下这里需不需要这个break


    def empty_events(self):
        """ Empty the event flags """

        self.reset_flag = False
        self.timeout_flag = False
        self._received_reset.clear()



class Chassis:
    """ Class that defines the basic motion of chassis """

    def __init__(self, beep_pattern, intake:Intake, lidar:Lidar, 
                 event_handler:EventHandler):
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.yaw_tolerance = 1
        self.yaw_rate = 0
        self.time_global_start = time.time()
        self.imu_init_angle_offset = bot.get_imu_attitude_data()[2]
        self.turn_num = 0

        self._beep_pattern = beep_pattern
        self._intake = intake
        self._lidar = lidar
        self.event_handler = event_handler


    def measure_stationary_yaw_drift_rate(self, duration:int, plot=False):
        """ Measures the rate of drift in z-direction (yaw rate) 

        :param duration: duration of measurement in seconds (integer)
        :return: yaw angle drift rate in degrees per second """

        yaws=[]
        times=[]
        self.imu_init_angle_offset = bot.get_imu_attitude_data()[2]

        start_time = time.time()
        yaw = bot.get_imu_attitude_data()[2]
        end_time = time.time()
        
        while end_time - start_time < duration:
            end_time = time.time()
            yaw_end = bot.get_imu_attitude_data()[2]
            if plot:
                times.append(end_time-start_time)
                yaws.append(yaw_end)

        if plot:
            plt.plot(times,yaws)
            plt.savefig('plt_no_offset.png')
        
        yaw_rate = (yaw_end - yaw)/(end_time-start_time)
        if yaw_rate == 0:
            print('IMU IS NOT READING')
            try:
                self._beep_pattern('-....')
            except:
                pass

        self.yaw_rate = yaw_rate

        return yaw_rate

    
    def get_yaw_calibrated(self):
        """ Calibrate yaw readings. 
        :return: calibrated yaw """

        t = time.time()
        yaw = bot.get_imu_attitude_data()[2]
        yaw = yaw - (t - self.time_global_start) * self.yaw_rate
        return yaw


    def action(self, controller, yaw_start, eat = True):
        """ Set motion of robot with PID control. 
        :param controller: a simple-pid PID controller, yaw_start: initial yaw angle """
        
        self.event_handler.check_reset() ###
        yaw = self.get_yaw_calibrated()
        # The following three lines is commented out for Main_3.py (Max)
        # control = controller(yaw)
        # error = yaw_start - yaw 
        # self.vz = max(-10, min(control, 10))
        # print("Error: {}, Control: {}, Vz: {}".format(error, control, self.vz))
        bot.set_car_motion(self.vx, self.vy, self.vz)
        if eat:
            self._intake.eat()
        else:
            self._intake.unload()
        # time.sleep(0.05)


    def stop(self):
        """ Stop all wheels and intake. """

        bot.set_car_motion(0, 0 ,0)
        self._intake.set_free_drive()


    # May not need
    def find_base(self):
        """ Find base from current heading. 
        :return: point: coordinate of base w.r.t robot """
        
        # Input: 4 sets of xy coordinates - points
        # Might have to make Lidar an attribute of Chassis Class, and call methods of Lidar
        points = self._lidar.get_last_arena_vertices()        
        if self.turn_num % 4 == 0: # Base in Quadrant 3
            for point in points:
                if point[0] < 0 and point[1] < 0:
                    return point
        elif self.turn_num % 4 == 1: # Base in Quadrant 2
            for point in points:
                if point[0] < 0 and point[1] > 0:
                    return point
        elif self.turn_num % 4 == 2: # Base in Quadrant 1
            for point in points:
                if point[0] > 0 and point[1] > 0:
                    return point
        elif self.turn_num % 4 == 3:
            for point in points: # Base in Quadrant 4
                if point[0] > 0 and point[1] < 0:
                    return point
        print('Base:', point)
    
    def find_base_no_turn_version(self):
        points = self._lidar.get_last_arena_vertices()
        for point in points:
            if point[0] < 0 and point[1] < 0:
                return point

    def revert_orientation(self):
        """ Align the orientation of the robot for returning. """
        num = self.turn_num - 2
        print(num)
        if num%4 == 3:
            self.turn(90)
            time.sleep(0.5)
        else:
            for i in range((num-2)%4):
                self.turn(-90)
                time.sleep(0.5)
        shift = self._lidar.get_last_angle_error()
        self.turn(-shift)



pass
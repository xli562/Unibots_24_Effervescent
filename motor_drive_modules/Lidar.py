import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
try:
    from Chassis import Buzzer
except:
    pass

import subprocess, threading, re, time
import numpy as np
from typing import List, Tuple


class Lidar:
    _instance = None    # Needed for the __new__() method

    def __init__(self):
        self._blocking = False
        self._last_reading = np.array([0., 0., 0.])
        # A list of fixed length 450 to store data for the last complete cycle
        self._last_cycle_readings = np.array([[0., 0., 0.]])
        # A list of growing length to store data for the current cycle
        self._current_cycle_readings = np.array([[0., 0., 0.]])
        self._last_fitted_angle_error = 0
        self._last_fitted_arena_vertices = np.array([[0., 0.], [0., 0.], [0., 0.], [0., 0.]])
        self._start_autoreceive_readings_thread()
        self._check_connection(timeout=5)
        self._new_data_available = False
        # Threads management
        self._threads = []
        self._terminate_all_threads = threading.Event()
        self._terminate_all_threads.clear()
    

    def __new__(cls, *args, **kwargs):
        """ To prevent the Lidar instance from being created
        multiple times. """

        if not cls._instance:
            cls._instance = super(Lidar, cls).__new__(cls, *args, **kwargs)
        return cls._instance


    def __del__(self):
        self._terminate_all_threads.set()
        for thread in self._threads:
            thread.join()
        try:
            buzzer = Buzzer()
            buzzer.beep_pattern('....  .. -')
        except:
            pass


    def _check_connection(self, timeout=5):
        """ Checks the Lidar connection.
        If the last reading is still empty after 5 seconds, 
        buzz the buzzer in a specific pattern if buzzer is connected. 
        
        :param timeout: the wait time before checking if lidar returns readings"""

        def check_reading():
            while not self._terminate_all_threads.is_set():
                if self._last_reading.size == 0:
                    try:
                        buzzer = Buzzer()
                        buzzer.beep_pattern('....  .   .  ', 5)
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
                        max_x_y = np.max(np.max(np.abs(readings_x)), np.max(np.abs(readings_y)))
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



        
# Example usage
if __name__ == "__main__":
    lidar = Lidar()
    print(lidar._last_reading)
    lidar.plot_fit_square()
    # while 1:
        # square_corners = lidar.fit_square()
        # print(lidar._last_cycle_readings)
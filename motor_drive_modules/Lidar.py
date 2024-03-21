from Plotter import Plotter
# from Chassis import Buzzer

import subprocess, threading, re
import numpy as np
from typing import List, Tuple


class Lidar:
    _instance = None    # Needed for the __new__() method

    def __init__(self):
        self._blocking = False
        self._last_reading = np.array([])
        # A list of fixed length 450 to store data for the last complete cycle
        self._last_cycle_readings = np.array([])
        # A list of growing length to store data for the current cycle
        self._current_cycle_readings = np.array([])
        self._start_autoreceive_readings_thread()
        self._check_connection(timeout=5)
        self._new_data_available = False
    

    def __new__(cls, *args, **kwargs):
        """ To prevent the Lidar instance from being created
        multiple times. """

        if not cls._instance:
            cls._instance = super(Lidar, cls).__new__(cls, *args, **kwargs)
        return cls._instance


    def _check_connection(self, timeout=5):
        """ Checks the Lidar connection.
        If the last reading is still empty after 5 seconds, 
        Buzz the buzzer in a specific pattern. """

        def check_reading():
            if not self._last_reading:
                # buzzer = Buzzer()
                # buzzer.beep_pattern('....  .   .  ', 5)
                pass
        threading.Timer(timeout, check_reading).start()


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
            # Pattern for regex parsing
            pattern = re.compile(r'\[(\d+): ([\d.]+), ([\d.]+)\]')
            for line in iter(pipe.readline, ''):
                match = pattern.search(line)
                if match:
                    index = int(match.group(1))
                    r = float(match.group(2))  # Radius (distance)
                    theta = float(match.group(3))  # Angle
                    self._last_reading = np.array([index, r, theta])
                    self._current_cycle_readings = np.append(self._current_cycle_readings, self._last_reading)
                    if index == 0:
                        # Update fresh data into last-cycle-reading
                        self._last_cycle_readings = self._current_cycle_readings
                        # Reset current cycle for each new cycle
                        self._current_cycle_readings = np.array([])

        # Data queue and thread setup
        stdout_thread = threading.Thread(target=capture_output, 
                                         args=(self.process.stdout,))
        stdout_thread.start()


    def get_last_cycle_readings(self) -> np.ndarray:
        """ This function reads from a cached variable
        and takes minimal processing effort 

        :return: an np.ndarray of the last cycle's readings"""
        return self._last_cycle_readings
    

    def get_last_reading(self) -> np.ndarray:
        """ This function reads from a cached variable
        and takes minimal processing effort 

        :return: an array of [index, r, theta] of the last point being read. """
        return self._last_reading


    def _polar_to_cartesian(self, r: float, theta: float) -> np.ndarray:
        """ Convert polar coordinates to cartesian coordinates. """
        x = r * np.cos(np.radians(theta))
        y = r * np.sin(np.radians(theta))
        return np.array([x, y])


    def _nomalise_points(self, points:np.ndarray, center:np.ndarray) -> np.ndarray:
        """ Normalises the point cloud to center at the given point.

        :param points: List[Tuple[float, float]] in Cartesians.
        :param centre: The centre's coord.s in Cartesians. """

        return points - center


    def _find_bounding_sides(self, points:np.ndarray) -> np.ndarray:
        """ Find the bounding box to the given points. 

        :returns: Min and max x and y values as below.
            np.array([min_x, min_y, max_x, max_y]) """
        
        if points:
            min_x = np.min(points[:, 0])
            max_x = np.max(points[:, 0])
            min_y = np.min(points[:, 1])
            max_y = np.max(points[:, 0])
        else:
            min_x, min_y, max_x, max_y = (0,0,0,0)
        return np.array([min_x, min_y, max_x, max_y])


    def fit_square(self, reduce_data_size_step=2, angle_step=1) -> list:
        """ Finds the rotated minimum-area rectangle of the point cloud.
         
        :param reduce_data_size_step: step for data size reduction
        :param angle_step: step for angle search 
        :return: list of 4 vertices of the min. area rect."""
        
        # Preprocess the data
        i = 0
        working_data = []   # The data points to be considered
        for _, r, theta in self.get_last_cycle_readings():
            # Reduce data set size for faster computation
            if i % reduce_data_size_step == 0:
                continue
            working_data.append(self._polar_to_cartesian(r, theta))
            i += 1
        
        # TODO:
        areas = []  # List to hold areas of the min. area rect.s

        
# Example usage
if __name__ == "__main__":
    lidar = Lidar()
    print(lidar._last_reading)
    plotter = Plotter()
    plotter.plot_lidar_fit_square(lidar.fit_square, lidar.get_last_cycle_readings, (-0.5, 0.5))
    # while 1:
        # square_corners = lidar.fit_square()
        # print(lidar._last_cycle_readings)
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
                    self._current_cycle_readings = np.vstack((self._current_cycle_readings, self._last_reading))
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
        if self._last_cycle_readings.size > 0:
            return self._last_cycle_readings
        else:
            return np.array([[0, 0., 0.]])
    

    def get_last_reading(self) -> np.ndarray:
        """ This function reads from a cached variable
        and takes minimal processing effort 

        :return: an array of [index, r, theta] of the last point being read. """
        if self._last_reading.size  > 0:
            return self._last_reading
        else:
            return np.array([0, 0., 0.])


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
        
        if points.size > 0:
            min_x = np.min(points[:, 0])
            max_x = np.max(points[:, 0])
            min_y = np.min(points[:, 1])
            max_y = np.max(points[:, 1])
        else:
            min_x, min_y, max_x, max_y = (0., 0., 0., 0.)
        return np.array([min_x, max_x, min_y, max_y])


    def fit_square(self, reduce_data_size_step=2, 
                   angle_step=1, sagging_index=0.2) -> list:
        """ Finds the rotated minimum-area rectangle (MAR) of the point cloud.
         
        :param reduce_data_size_step: step for data size reduction
        :param angle_step: step for angle search, max is 10
        :return: rotation_angle, bounding_vertices
            rotation_angle: amount of anticlockwise rotation of the fit;
            bounding_vertices: np.ndarray of 4 vertices of the min. area rect,
            see below for order.
            (max_x, max_y), (max_x, min_y), (min_x, min_y), (min_x, max_y) """
        
        # Foolproof inputs
        angle_step = max(0, min(angle_step, 10))

        # Preprocess the data
        # TODO: filter out very-far points
        i = 0
        cleaned_data = np.array([])   # The data points to be considered
        for _, r, theta in self.get_last_cycle_readings():
            # Reduce data set size for faster computation
            if i % reduce_data_size_step == 0:
                continue
            cleaned_data = np.vstack((cleaned_data, self._polar_to_cartesian(r, theta)))
            i += 1
        if cleaned_data.size == 0:
            cleaned_data = np.array([0., 0., 0., 0.])
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
        sagging_iter_bound = 20 // angle_step
        sagging_coefficient = 1 + sagging_index * angle_step
        for rotation_angle in range(angle_step, 91, angle_step):
            np.vstack((rotation_angles, rotation_angle))
            rotation_matrix = np.array([[np.cos(np.radians(rotation_angle)), -np.sin(np.radians(rotation_angle))],
                                       [np.sin(np.radians(rotation_angle)), np.cos(np.radians(rotation_angle))]])
            # Apply rotation to every point            
            rotated_data = cleaned_data @ rotation_matrix.T
            bounding_sides = self._find_bounding_sides(rotated_data)
            bounding_sides_array = np.vstack((bounding_sides_array, bounding_sides))
            areas = np.vstack((areas, calc_MAR_area(bounding_sides)))
            # Stop if the sagging point of the function is found
            if i >= sagging_iter_bound:
                if areas[i] > areas[i-sagging_iter_bound] * sagging_coefficient:
                    rotation_angle -= sagging_iter_bound * angle_step
                    bounding_sides = bounding_sides_array[i-sagging_iter_bound]
                    bounding_vertices = np.array([bounding_sides[1], bounding_sides[3]],
                                                 [bounding_sides[1], bounding_sides[2]],
                                                 [bounding_sides[0], bounding_sides[2]],
                                                 [bounding_sides[0], bounding_sides[3]])
                    bounding_vertices = (bounding_vertices @ rotation_matrix.T) + center_offset
                    return rotation_angle, bounding_vertices
            i += 1
        i = np.argmin(areas)
        rotation_angle = angle_step * i
        bounding_sides = bounding_sides_array[i]
        bounding_vertices = np.array([bounding_sides[1], bounding_sides[3]],
                                    [bounding_sides[1], bounding_sides[2]],
                                    [bounding_sides[0], bounding_sides[2]],
                                    [bounding_sides[0], bounding_sides[3]])
        bounding_vertices = (bounding_vertices @ rotation_matrix.T) + center_offset
        return rotation_angle, bounding_vertices



        
# Example usage
if __name__ == "__main__":
    lidar = Lidar()
    print(lidar._last_reading)
    plotter = Plotter()
    plotter.plot_lidar_fit_square(lidar.fit_square, lidar.get_last_cycle_readings, (-0.5, 0.5))
    # while 1:
        # square_corners = lidar.fit_square()
        # print(lidar._last_cycle_readings)
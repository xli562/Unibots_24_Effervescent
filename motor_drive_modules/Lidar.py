from Chassis import Buzzer

import subprocess, threading, re
import numpy as np
from typing import List, Tuple


class Lidar:
    def __init__(self):
        self._blocking = False
        self._last_reading = ()
        # A list of fixed length 450 to store data for the last complete cycle
        self._last_cycle_readings = []
        # A list of growing length to store data for the current cycle
        self._current_cycle_readings = []
        self._start_autoreceive_readings_thread()
        self._check_connection(timeout=5)
    

    def _check_connection(self, timeout=5):
        """ Checks the Lidar connection.
        If the last reading is still empty after 5 seconds, 
        Buzz the buzzer in a specific pattern. """
        def check_reading():
            if not self._last_reading:
                buzzer = Buzzer()
                buzzer.beep_pattern('....  .   .  ', 5)
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
                    self._last_reading = (index, r, theta)
                    self._current_cycle_readings.append(self._last_reading)
                    if index == 0:
                        # Update fresh data into last-cycle-reading
                        self._last_cycle_readings = self._current_cycle_readings
                        # Reset current cycle for each new cycle
                        self._current_cycle_readings = []

        # Data queue and thread setup
        stdout_thread = threading.Thread(target=capture_output, 
                                         args=(self.process.stdout,))
        stdout_thread.start()

    def get_one_reading(self) -> Tuple[int, float, float]:
        """ Returns a tuple of (measurement_index, distance, angle). """
        # TODO: 2024/03/19: tidy up this function
        cpp_file_folder = './lidar_sdk/build'

        # Function to capture output from stdout
        def capture_output(pipe, label):
            for line in iter(pipe.readline, ''):
                print(f"{label}: {line}", end='')
        
        # Start the process
        process = subprocess.Popen(cpp_file, stdout=subprocess.PIPE, 
                                   stderr=subprocess.PIPE, bufsize=1, 
                                   universal_newlines=True, 
                                   cwd=cpp_file_folder, shell=True)

        # Create threads to handle stdout and stderr output
        stdout_thread = threading.Thread(target=capture_output, 
                                         args=(process.stdout))

        # Start the threads
        stdout_thread.start()

        # Wait for the output capture threads to finish (if the process is continuous, this might never happen without an external stop condition)
        stdout_thread.join()

        # Optionally, add a mechanism to terminate the process safely
        # process.terminate()

        return [(i, np.random.uniform(0.02, 12.0), np.random.uniform(0.0, 360.0)) for i in range(450)]


    def _polar_to_cartesian(self, r: float, theta: float) -> Tuple[float, float]:
        """ Convert polar coordinates to cartesian coordinates. """
        x = r * np.cos(np.radians(theta))
        y = r * np.sin(np.radians(theta))
        return (x, y)


    def _fit_square_edges(self, points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """  Fit a square to the given points. This is a simplified version 
        and might need adjustments based on actual Lidar data quality. """
        
        # Simplified approach: Find the bounding box and assume it's the square
        if points:
            min_x = min(points, key=lambda x: x[0])[0]
            max_x = max(points, key=lambda x: x[0])[0]
            min_y = min(points, key=lambda x: x[1])[1]
            max_y = max(points, key=lambda x: x[1])[1]
        else:
            min_x, min_y, max_x, max_y = (0,0,0,0)

        return [(min_x, min_y), (min_x, max_y), (max_x, max_y), (max_x, min_y)]


    def fit_square(self) -> List[Tuple[float, float]]:
        """ Reads from the lidar, and outputs the coordinates of the square arena's four corners
        in polar coordinates (r, theta). """
        filtered_points = self._filter_robots(self._last_cycle_readings)  # Filter out robot readings
        square_corners_cartesian = self._fit_square_edges(filtered_points)  # Fit square edges

        # Convert corners back to polar coordinates
        square_corners_polar = [(np.sqrt(x**2 + y**2), np.degrees(np.arctan2(y, x))) for x, y in square_corners_cartesian]
        return square_corners_polar
    

    def _filter_robots(self, readings:List[Tuple[int, float, float]], 
                                    by_angle:bool=False,
                                    max_robot_angle=60, 
                                    min_group_size:int=5) -> List[Tuple[float, float]]:
        """ Filter out points that are likely to be robots based on angle coverage.
        Reads are grouped by continuity in angle; large gaps likely indicate non-robot objects (arena edges).
        
        :param readings: List of tuples (measurement_index, distance, angle) from 
                the Lidar.
        :param by_angle: Filter robots by max angle instead of by min point cluster.
        :param max_robot_angle: Maximum angle in degrees a robot can cover at 
                minimum distance.
        :return: Filtered list of points in cartesian coordinates, likely part 
                of the arena edges. """
        if by_angle: 
            # Convert readings to polar coordinates and sort by angle
            polar_points = sorted([(distance, angle) for _, distance, angle in readings], key=lambda x: x[1])
            
            # Group points by angle continuity, considering robot size
            groups = []
            current_group = [polar_points[0]]
            
            for i in range(1, len(polar_points)):
                distance, angle = polar_points[i]
                prev_distance, prev_angle = polar_points[i-1]
                
                # If the gap to the previous angle is small, it's likely part of the same object
                if angle - prev_angle < max_robot_angle:
                    current_group.append((distance, angle))
                else:
                    # If the gap is large, start a new group
                    groups.append(current_group)
                    current_group = [(distance, angle)]
            
            # Add the last group if not empty
            if current_group:
                groups.append(current_group)
            
            # Filter out small groups, assuming they are robots
            filtered_groups = [group for group in groups if len(group) > 1]
            
            # Flatten the list of groups and convert to cartesian coordinates
            filtered_points = [polar_to_cartesian(distance, angle) for group in filtered_groups for distance, angle in group]
        else:
            # Convert all readings to cartesian coordinates
            points = [self._polar_to_cartesian(distance, angle) for _, distance, angle in readings]
            points.sort(key=lambda x: x[1])  # Sort points based on y-coordinate for grouping
            
            # Placeholder for filtered points
            filtered_points = []

            # Assuming robots are significantly smaller objects compared 
            # to the arena, they would form smaller clusters of points.
            # We identify larger clusters as parts of the arena.
            i = 0
            while i < len(points):
                cluster = [points[i]]
                j = i + 1
                # Cluster threshold, can be adjusted
                while j < len(points) and (points[j][1] - points[j - 1][1] < 0.1):  
                    cluster.append(points[j])
                    j += 1

                # If the cluster size is above a certain threshold, it's likely part of the arena
                if len(cluster) >= min_group_size:
                    filtered_points.extend(cluster)

                i = j
        return filtered_points


# Example usage
if __name__ == "__main__":
    lidar = Lidar()
    print(lidar._last_reading)
    # square_corners = lidar.fit_square()
    # print(square_corners)
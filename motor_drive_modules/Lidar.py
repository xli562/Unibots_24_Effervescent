from Plotter import Plotter
# from Chassis import Buzzer

import subprocess, threading, re
import numpy as np
from typing import List, Tuple


class Lidar:
    _instance = None    # Needed for the __new__() method

    def __init__(self):
        self._blocking = False
        self._last_reading = ()
        # A list of fixed length 450 to store data for the last complete cycle
        self._last_cycle_readings = []
        # A list of growing length to store data for the current cycle
        self._current_cycle_readings = []
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


    def get_last_cycle_readings(self) -> list:
        """ This function reads from a cached variable
        and takes minimal processing effort """
        return self._last_cycle_readings
    

    def get_last_reading(self) -> tuple:
        """ This function reads from a cached variable
        and takes minimal processing effort """
        return self._last_reading


    def _polar_to_cartesian(self, r: float, theta: float) -> Tuple[float, float]:
        """ Convert polar coordinates to cartesian coordinates. """
        x = r * np.cos(np.radians(theta))
        y = r * np.sin(np.radians(theta))
        return (x, y)


    def _filter_robots(self, readings:list, 
                       by_angle:bool=False, by_cluster:bool=True,
                       max_robot_angle=60, min_group_size:int=5) -> list:
        """ Filters out points that are likely to be robots.
        
        :param readings: List of tuples (measurement_index, distance, angle)
            from the Lidar.
        :param by_angle: Filter robots by max angle. Reads are grouped by 
            continuity in angle; large gaps likely indicate 
            non-robot objects (arena edges).
        :param max_robot_angle: Maximum angle in degrees a robot can cover at 
            minimum distance.
        :param by_cluster: Filter robots by min point cluster. 
        :return: Filtered list of points in Cartesians, likely part 
            of the arena edges. """
        if by_angle: 
            # Convert readings to polar coordinates and sort by angle
            polar_points = sorted([(distance, angle) for _, distance, angle in readings], key=lambda x: x[1])
            
            # Group points by angle continuity, considering robot size
            groups = []
            current_group = [polar_points[0]]
            
            for i in range(1, len(polar_points)):
                r, theta = polar_points[i]
                prev_r, prev_theta = polar_points[i-1]
                
                # If the gap to the previous angle is small, it's likely part of the same object
                if theta - prev_theta < max_robot_angle:
                    current_group.append((r, theta))
                else:
                    # If the gap is large, start a new group
                    groups.append(current_group)
                    current_group = [(r, theta)]
            
            # Add the last group if not empty
            if current_group:
                groups.append(current_group)
            
            # Filter out small groups, assuming they are robots
            filtered_groups = [group for group in groups if len(group) > 1]
            
            # Flatten the list of groups and convert to cartesian coordinates
            filtered_points = [self._polar_to_cartesian(distance, angle) for group in filtered_groups for distance, angle in group]
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


    def _find_square_center(self, points:list) -> Tuple[float, float]:
        """ Estimate the center of the square by averaging the given points. 
        :param points: List[Tuple[float, float]] in Cartesians """
        x_coords, y_coords = zip(*points)
        center_x = sum(x_coords) / len(x_coords)
        center_y = sum(y_coords) / len(y_coords)
        return center_x, center_y


    def _calculate_angle(self, center:Tuple[float, float], 
                         point:Tuple[float, float]) -> float:
        """ Calculate the angle of the square (in radians)
        from the center to a point. """
        dx, dy = point[0] - center[0], point[1] - center[1]
        return np.arctan2(dy, dx)


    def _cluster_readings(self, points:list) -> list:
        """ Cluster the points based on which side of the square they are in. 
        :param points: List[Tuple[float, float]] in Cartesians
        :return: List[List[Tuple[float, float]] * 4] four clussters of points
            in Cartesians """
        
        center = self._find_square_center(points)
        angles = [self._calculate_angle(center, point) for point in points]
        
        # Divide the full circle into 4 sectors based on angles, allowing for some overlap
        # influenced by the scatter_coefficient
        clusters = [[] for _ in range(4)]
        sector_size = np.pi / 2  # 90 degrees in radians
        
        for point, angle in zip(points, angles):
            sector_index = int((angle % (2 * np.pi)) / sector_size)
            clusters[sector_index].append(point)
        
        return clusters


    def _find_bounding_box(self, points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """  Fit a square to the given points.
        Currently finds the bounding box """
        if points:
            min_x = min(points, key=lambda x: x[0])[0]
            max_x = max(points, key=lambda x: x[0])[0]
            min_y = min(points, key=lambda x: x[1])[1]
            max_y = max(points, key=lambda x: x[1])[1]
        else:
            min_x, min_y, max_x, max_y = (0,0,0,0)

        return [(min_x, min_y), (min_x, max_y), (max_x, max_y), (max_x, min_y)]

    def fit_square(self, reduce_data_size_step=2, angle_step=1) -> list:
        """ Finds the rotated minimum-area rectangle of the point cloud.
         
        :param reduce_data_size_step: step for data size reduction
        :param angle_step: step for angle search 
        :return: list of 4 vertices of the min. area rect."""
        i = 0
        working_data = []   # The data points to be considered
        for _, r, theta in self.get_last_reading():
            # Reduce data set size for faster computation
            if i % reduce_data_size_step == 0:
                continue
            working_data.append(self._polar_to_cartesian(r, theta))
            i += 1
        
        areas = []  # List to hold areas of the min. area rect.s

        


    def old_fit_square(self) -> List[Tuple[float, float]]:
        """ Reads from the lidar, and outputs the coordinates of the square arena's four corners
        in polar coordinates (r, theta). """
        # filtered_points = self._filter_robots(self._last_cycle_readings)  # Filter out robot readings
        filtered_points = []
        for reading in self._last_cycle_readings:
            r = reading[1]
            theta = reading[2]
            filtered_points.append(self._polar_to_cartesian(r, theta))
        # print(filtered_points)
        square_corners_cartesian = self._find_bounding_box(filtered_points)  # Fit square edges

        # Convert corners back to polar coordinates
        return square_corners_cartesian        
        square_corners_polar = [(np.sqrt(x**2 + y**2), np.degrees(np.arctan2(y, x))) for x, y in square_corners_cartesian]
        # return square_corners_polar
    


# Example usage
if __name__ == "__main__":
    lidar = Lidar()
    print(lidar._last_reading)
    plotter = Plotter()
    plotter.plot_lidar_fit_square(lidar.fit_square, lidar.get_last_cycle_readings, (-0.5, 0.5))
    # while 1:
        # square_corners = lidar.fit_square()
        # print(lidar._last_cycle_readings)
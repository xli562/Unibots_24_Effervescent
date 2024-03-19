import numpy as np
from typing import List, Tuple

import subprocess
import threading


class Lidar:
    def __init__(self):
        self._blocking = False


    def get_one_reading(self) -> Tuple[int, float, float]:
        """ Returns a tuple of (measurement_index, distance, angle). """
        # TODO: 2024/03/19: tidy up this function
        cpp_file_folder = './lidar_sdk/build'
        cpp_file = './blocking_test' if self._blocking else './non-blocking_test'

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
                                         args=(process.stdout, 'STDOUT'))
        stderr_thread = threading.Thread(target=capture_output, 
                                         args=(process.stderr, 'STDERR'))

        # Start the threads
        stdout_thread.start()
        stderr_thread.start()

        # Wait for the output capture threads to finish (if the process is continuous, this might never happen without an external stop condition)
        stdout_thread.join()
        stderr_thread.join()

        # Optionally, add a mechanism to terminate the process safely
        # process.terminate()

        return [(i, np.random.uniform(0.02, 12.0), np.random.uniform(0.0, 360.0)) for i in range(450)]


    def _polar_to_cartesian(self, r: float, theta: float) -> Tuple[float, float]:
        """ Convert polar coordinates to cartesian coordinates. """
        x = r * np.cos(np.radians(theta))
        y = r * np.sin(np.radians(theta))
        return (x, y)


    def _fit_square_edges(self, points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """  Fit a square to the given points. This is a simplified version and might need
        adjustments based on actual Lidar data quality. """
        
        # Simplified approach: Find the bounding box and assume it's the square
        min_x = min(points, key=lambda x: x[0])[0]
        max_x = max(points, key=lambda x: x[0])[0]
        min_y = min(points, key=lambda x: x[1])[1]
        max_y = max(points, key=lambda x: x[1])[1]

        return [(min_x, min_y), (min_x, max_y), (max_x, max_y), (max_x, min_y)]


    def fit_square(self) -> List[Tuple[float, float]]:
        """ Reads from the lidar, and outputs the coordinates of the square arena's four corners
        in polar coordinates (r, theta). """
        readings = [self.get_one_reading() for _ in range(450)]  # Simulate multiple Lidar readings
        filtered_points = self.filter_robots(readings)  # Filter out robot readings
        square_corners_cartesian = self._fit_square_edges(filtered_points)  # Fit square edges

        # Convert corners back to polar coordinates
        square_corners_polar = [(np.sqrt(x**2 + y**2), np.degrees(np.arctan2(y, x))) for x, y in square_corners_cartesian]
        return square_corners_polar
    

    def _filter_robots_by_max_angle(self, readings:list, max_robot_angle=60) -> list:
        """ Filter out points that are likely to be robots based on angle coverage.
        Reads are grouped by continuity in angle; large gaps likely indicate non-robot objects (arena edges).
        
        :param readings: List of tuples (measurement_index, distance, angle) from the Lidar.
        :param max_robot_angle: Maximum angle in degrees a robot can cover at minimum distance.
        :return: Filtered list of points in cartesian coordinates, likely part of the arena edges. """
        
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
        
        return filtered_points


    def _filter_robots_by_min_point_cluster(self, readings: List[Tuple[int, float, float]], min_group_size: int = 5) -> List[Tuple[float, float]]:
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
    square_corners = lidar.fit_square()
    print(square_corners)
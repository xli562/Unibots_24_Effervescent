import subprocess
import threading
import numpy as np
import re


# Thread function for capturing output
def capture_output(pipe, data_queue):
    pattern = re.compile(r'\[(\d+): ([\d.]+), ([\d.]+)\]')
    for line in iter(pipe.readline, ''):
        match = pattern.search(line)
        if match:
            index = int(match.group(1))
            new_r = float(match.group(2))  # Radius (distance)
            new_theta = np.radians(float(match.group(3)))  # Angle, converted to radians
            data_queue.append((new_theta, new_r))  # Append tuple to queue
            print(f"Data: [Index: {index}, Distance: {new_r}, Angle: {new_theta}]\n", end = '')

# The path to the directory where you want to run the command
cpp_file_folder = '/home/eff/Desktop/Unibots_24_Effervescent/lidar_sdk/build'
# The command you want to run
cpp_file = './blocking_test'  # Ensure this is executable in the target directory
process = subprocess.Popen(cpp_file, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=1, universal_newlines=True, cwd=cpp_file_folder, shell=True)

# Data queue and thread setup
data_queue = []
stdout_thread = threading.Thread(target=capture_output, args=(process.stdout, data_queue))
stdout_thread.start()

stdout_thread.join()
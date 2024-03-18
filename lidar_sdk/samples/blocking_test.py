import subprocess
import threading
import matplotlib.pyplot as plt
import numpy as np
import re
import time

# Initialize matplotlib in interactive mode
plt.ion()

# Setup the polar plot
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
r_data, theta_data = [], []
line, = ax.plot(theta_data, r_data)

# Function to update plot (called from the main thread)
def update_plot(new_theta, new_r):
    r_data.append(new_r)
    theta_data.append(new_theta)
    line.set_xdata(theta_data)
    line.set_ydata(r_data)
    ax.relim()
    ax.autoscale_view(True, True, True)
    fig.canvas.draw()
    fig.canvas.flush_events()

# Thread function for capturing output
def capture_output(pipe, data_queue):
    pattern = re.compile(r'\[\d+: ([\d.]+), ([\d.]+)\]')
    for line in iter(pipe.readline, ''):
        match = pattern.search(line)
        if match:
            new_r = float(match.group(1))  # Radius (distance)
            new_theta = np.radians(float(match.group(2)))  # Angle, converted to radians
            data_queue.append((new_theta, new_r))  # Append tuple to queue

# The path to the directory where you want to run the command
working_directory = '/home/eff/Desktop/Unibots_24_Effervescent/lidar_sdk/build'
# The command you want to run
command = './non-blocking_test'  # Ensure this is executable in the target directory
process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=1, universal_newlines=True, cwd=working_directory, shell=True)

# Data queue and thread setup
data_queue = []
stdout_thread = threading.Thread(target=capture_output, args=(process.stdout, data_queue))
stdout_thread.start()

try:
    while process.poll() is None or stdout_thread.is_alive():
        # Process any new data in the queue
        while data_queue:
            new_theta, new_r = data_queue.pop(0)  # Get new data
            update_plot(new_theta, new_r)  # Update the plot
        time.sleep(0.1)  # Briefly sleep to yield control
finally:
    stdout_thread.join()
    plt.ioff()
    plt.show()

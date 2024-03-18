import subprocess
import threading
import matplotlib.pyplot as plt
import time

plt.ion()
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
r_data, theta_data = [], []
line, = ax.plot(theta_data, r_data, 'b-')  # 'b-' sets the line color to blue


# The path to the directory where you want to run the command
working_directory = '/home/eff/Desktop/Unibots_24_Effervescent/lidar_sdk/build'

# The command you want to run
command = './blocking_test'  # Ensure this is executable in the target directory

# Function to update the polar plot with new data
def update_plot(new_theta, new_r):
    theta_data.append(new_theta)
    r_data.append(new_r)
    line.set_xdata(theta_data)
    line.set_ydata(r_data)
    ax.relim()
    ax.autoscale_view(True, True, True)
    fig.canvas.draw()
    fig.canvas.flush_events()

# Function to capture and plot output on a polar plot
def capture_and_plot(pipe):
    pattern = re.compile(r'\[(\d+): ([\d.]+), ([\d.]+)\]')
    for line in iter(pipe.readline, ''):
        match = pattern.search(line)
        if match:
            # No need for index in plotting, but it's parsed in case you need it
            index = int(match.group(1))
            new_r = float(match.group(2))  # Distance
            new_theta = float(match.group(3))  # Angle, assuming it's already in radians
            
            # If your angle is in degrees, convert to radians with np.radians
            # new_theta = np.radians(new_theta)
            
            update_plot(new_theta, new_r)

# Start the process
process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=1, universal_newlines=True, cwd=working_directory, shell=True)

# Create threads to handle stdout and stderr output
stdout_thread = threading.Thread(target=capture_and_plot, args=(process.stdout, 'STDOUT'))
stderr_thread = threading.Thread(target=capture_and_plot, args=(process.stderr, 'STDERR'))

# Start the threads
stdout_thread.start()
stderr_thread.start()

# Wait for the output capture threads to finish (if the process is continuous, this might never happen without an external stop condition)
stdout_thread.join()
stderr_thread.join()

# Optionally, add a mechanism to terminate the process safely
# process.terminate()

import subprocess
import threading

# The path to the directory where you want to run the command
working_directory = '/home/eff/Desktop/Unibots_24_Effervescent/lidar_sdk/build'

# The command you want to run
command = './non-blocking_test'  # Ensure this is executable in the target directory

# Function to capture output from stdout
def capture_output(pipe, label):
    for line in iter(pipe.readline, ''):
        print(f"{label}: {line}", end='')

# Start the process
process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=1, universal_newlines=True, cwd=working_directory, shell=True)

# Create threads to handle stdout and stderr output
stdout_thread = threading.Thread(target=capture_output, args=(process.stdout, 'STDOUT'))
stderr_thread = threading.Thread(target=capture_output, args=(process.stderr, 'STDERR'))

# Start the threads
stdout_thread.start()
stderr_thread.start()

# Wait for the output capture threads to finish (if the process is continuous, this might never happen without an external stop condition)
stdout_thread.join()
stderr_thread.join()

# Optionally, add a mechanism to terminate the process safely
# process.terminate()

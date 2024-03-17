import subprocess
import sys

# Assuming the executable is named 'ordlidar_driver' or 'ordlidar_driver.exe'
executable_name = "ordlidar_driver.exe" if sys.platform.startswith("win") else "./ordlidar_driver"

# Start the subprocess and attach to its output stream
process = subprocess.Popen(
    executable_name,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    universal_newlines=True
)

try:
    # Read output from the process
    for line in iter(process.stdout.readline, ''):
        print(line, end='')

except KeyboardInterrupt:
    print("Terminating the process...")
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        print("Force killing the process...")
        process.kill()

    print("Process terminated.")

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Make sure to clean up and close any streams if necessary
    process.stdout.close()
    process.stderr.close()

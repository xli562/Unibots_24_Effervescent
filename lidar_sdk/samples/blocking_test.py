import subprocess

# Compile the C++ program first if needed
# subprocess.run(['g++', 'blocking_test.cpp', '-o', 'blocking_test', '-lyour_library'])

# The command to run the C++ program
command = './blocking_test'

# Using subprocess.Popen to execute the command and capture output
with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True) as process:
    try:
        # Wait for the program to finish and capture stdout and stderr
        stdout, stderr = process.communicate(timeout=30)  # Adjust the timeout as necessary
    except subprocess.TimeoutExpired:
        process.kill()
        stdout, stderr = process.communicate()
        print("The program didn't finish in the allotted time.")

    print("STDOUT:")
    print(stdout)
    print("STDERR:")
    print(stderr)

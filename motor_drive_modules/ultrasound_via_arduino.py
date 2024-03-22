import serial
import threading
import time
import struct
import subprocess
import sys


def ROBOT_RESET():
    print("Restarting...")
    python = sys.executable
    subprocess.call([python, "obstacle_avoidance.py"])
    sys.exit()


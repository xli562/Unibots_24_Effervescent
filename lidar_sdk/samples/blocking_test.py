import signal
import time
import sys
import ordlidar

# Handler for signal interruption (SIGINT)
def sig_handle(signo, frame):
    print(f"Program exit, Receive SIGNAL {signo}")
    global running
    running = False
    time.sleep(1)
    sys.exit(1)

# Register the SIGINT handler
signal.signal(signal.SIGINT, sig_handle)

# Main program logic
if __name__ == "__main__":
    running = True

    # Constants representing the lidar type and model
    type = ordlidar.ORADAR_TYPE_SERIAL
    model = ordlidar.ORADAR_MS200

    # Create an instance of OrdlidarDriver from the ordlidar module
    device = ordlidar.OrdlidarDriver(type, model)
    
    # Port name and baudrate configuration
    if sys.platform.startswith('win'):
        port_name = "COM18"
    else:
        port_name = "/dev/ttyACM0"
    serial_baudrate = 230400
    is_logging = True

    count = 0
    device.set_serial_port(port_name, serial_baudrate)

    while running:
        if device.connect():
            print("Lidar device connected successfully.")
            break
        else:
            print(f"Lidar device connection to {port_name} failed.")
            time.sleep(1)

    while running:
        # Assuming the method grab_full_scan_blocking correctly returns a tuple (ret, scan_data)
        ret, scan_data = device.grab_full_scan_blocking(1000)
        if ret:
            count += 1
            print(f"Count = {count}, Point num: {len(scan_data.data)}")
            if is_logging:
                for i, point in enumerate(scan_data.data):
                    print(f"[{i}: Distance={point.distance}, Angle={point.angle}]")
        else:
            print("Error: Failed to get full scan data.")

    device.disconnect()

import signal
import sys
import time
import ordlidar

running = True

def sig_handle(signo, frame):
    global running
    print(f"Program exit, receive SIGNAL {signo}")
    running = False
    time.sleep(1)
    sys.exit(1)

# Register the SIGINT handler
signal.signal(signal.SIGINT, sig_handle)

if __name__ == "__main__":
    type = ordlidar.ORADAR_TYPE_SERIAL
    model = ordlidar.ORADAR_MS200

    # Initialize the OrdlidarDriver with the specified type and model
    device = ordlidar.OrdlidarDriver(type, model)

    # Port name and baud rate settings
    if sys.platform.startswith('win'):
        port_name = "COM18"
    else:
        port_name = "/dev/ttyACM0"
    serial_baudrate = 230400
    is_logging = True

    count = 0
    device.set_serial_port(port_name, serial_baudrate)

    # Attempt to connect to the device
    while running:
        if device.connect():
            print("Lidar device connected successfully.")
            break
        else:
            print("Lidar device connection failed.")
            time.sleep(1)

    # Main loop to grab and process scan data
    while running:
        ret, scan_data = device.grab_full_scan()
        if ret:
            count += 1
            print(f"Count = {count}, Point num: {len(scan_data.data)}")
            if is_logging:
                for i, point in enumerate(scan_data.data):
                    # Assuming distance is in millimeters and converting to meters for display
                    print(f"[{i}: {point.distance * 0.001}, {point.angle}]")
            # Slight delay to prevent overloading the CPU
            time.sleep(0.1)
        else:
            print("Failed to retrieve full scan data.")

    # Disconnect the device before exiting
    device.disconnect()

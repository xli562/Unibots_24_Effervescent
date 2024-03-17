

    
import serial
import threading
import time
import struct

class Ultrasound:
    def __init__(self, com_port="/dev/ttyUSB0", baud_rate=115200):
        self.ser = serial.Serial(com_port, baud_rate)
        self.distances = [0] * 5  # Array to store 5 distances
        self.updated_sensors = [False]*5
        self.updated = False # 用来判断是否所有UltrasoundSensor都更新了最新读数
        # 当不加入Threading，读取数据时会永远停留在receive_distances的while循环中。故需要加入Thread
        # 先测试阶段，先不放入Thread
        # 如果测试没问题，加入Thread不单单要uncomment下面两行，还需要添加别的代码（later）
        # self.running = False
        # self.receive_thread = None

    def check_update_status(self):
        for status in self.updated_sensors:
            if status == False:
                return False
        return True

    def receive_distances(self):
        self.updated = False
        self.updated_sensors = [False]*5
        while not(self.updated):
            data_str = self.ser.readline().strip().decode('utf-8')
            readings = data_str.split(',')
            for reading in readings:
                if reading.startswith("U:"):  # Ultrasonic sensor reading
                    ultrasonic_distance = int(reading.split(':')[1])
                    sensor_index = int(reading.split(':')[0][1])
                    self.update_distance(ultrasonic_distance, sensor_index)
                    self.updated_sensors[sensor_index] = True

                    if(self.check_update_status()):
                        self.updated = True
    


    def update_distance(self, distance, sensor_index):
        self.distances[sensor_index] = distance

    def get_distances(self):
        return self.distances


# Example usage:
if __name__ == "__main__":
    ultrasound = Ultrasound()  # Create an Ultrasound object
    while True:
        distances = ultrasound.receive_distances()
        distances = ultrasound.get_distances()  
        print("Distances:", distances)
        time.sleep(0.1)  


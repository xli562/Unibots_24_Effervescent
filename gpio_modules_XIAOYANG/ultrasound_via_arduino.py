

    
import serial
import threading
import time
import struct

class Ultrasound:
    def __init__(self, com_port="/dev/ttyACM1", baud_rate=115200):
        self.ser = serial.Serial(com_port, baud_rate)
        self.distances = [0] * 5  # Left_Bottom, Left_Top, Right, Front, Back
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
                if reading.startswith("U"):  # Ultrasonic sensor reading
                    ultrasonic_distance = int(reading.split(':')[1])
                    sensor_index = int(reading.split(':')[0][1])
                    self.update_distance(ultrasonic_distance, sensor_index)
                    self.updated_sensors[sensor_index] = True

                    if(self.check_update_status()):
                        self.updated = True

    def update_distance(self, distance, sensor_index):
        self.distances[sensor_index] = distance

    def get_distances(self):
        if (self.updated):
            return self.distances
        else:
            self.receive_distances()

    @property
    def rugby_left(self):
        if (self.updated):
            return (self.distances[1] - self.distances[0]) >= 5
            # rugby_left is True when the difference between top and bottom sensor > 5cm

    @property
    def object_left(self):
        return self.distances[1] < 10
        # when object detected within 10cm, object detected

    @property
    def object_right(self):
        return self.distances[2] < 10
        # when object detected within 10cm, object detected
    
    @property
    def object_front(self):
        return self.distances[3] < 10
        # when object detected within 10cm, object detected
    
    @property
    def object_back(self):
        return self.distances[3] < 10
        # when object detected within 10cm, object detected

ultrasound = Ultrasound()
while True:
    distances = ultrasound.receive_distances()
    distances = ultrasound.get_distances()  
    print("Distances:", distances)
    time.sleep(0.1)  

# # Example usage:
# if __name__ == "__main__":
#     ultrasound = Ultrasound()  # Create an Ultrasound object
#     while True:
#         distances = ultrasound.receive_distances()
#         distances = ultrasound.get_distances()  
#         print("Distances:", distances)
#         time.sleep(0.1)  




    
import serial
import threading
import time
import struct

NUM_OF_ULTRASOUND_SENSOR = 5

class Ultrasound:
    def __init__(self, com_port="/dev/ttyUSB0", baud_rate=115200):
        self.ser = serial.Serial(com_port, baud_rate)
        self.distances = [0] * NUM_OF_ULTRASOUND_SENSOR  # Left_Bottom, Right, Front, Back, Left_Top
        self.updated_sensors = [False]*NUM_OF_ULTRASOUND_SENSOR
        self.updated = False # 用来判断是否所有UltrasoundSensor都更新了最新读数
        self.obstacle_count = [0] * NUM_OF_ULTRASOUND_SENSOR #代表了连续几次iteration检测到了obstacle，为了避免随机触发
        self.rugby_count = 0 #同理，代表连续几次检测到了rugby
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
        self.updated_sensors = [False] * NUM_OF_ULTRASOUND_SENSOR
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

            # Update Rugby连续几次iteration被检测到了            
            if (self.distances[4] - self.distances[0]) >= 5:
                self.rugby_count += 1
            else:
                self.rugby_count = 0

    def update_distance(self, distance, sensor_index):
        self.distances[sensor_index] = distance
        if (distance < 10 and distance > 0):
            self.obstacle_count[sensor_index] += 1
        else:
            self.obstacle_count[sensor_index] = 0

    def get_distances(self):
        if (self.updated):
            return self.distances
        else:
            self.receive_distances()

    @property
    def rugby_left(self):
        if (self.updated):
            return (self.rugby_count >= 5)
            # return (self.distances[4] - self.distances[0]) >= 5
            # rugby_left is True when the difference between top and bottom sensor > 5cm

    @property
    def object_left(self):
        return (self.obstacle_count[4] >= 5)
        # return (self.distances[4] < 10 and self.distances[4] > 0)
        # when object detected within 10cm, object detected

    @property
    def object_right(self):
        return (self.obstacle_count[1] >= 5)
        # return (self.distances[1] < 10 and self.distances[1] > 0)
        # when object detected within 10cm, object detected
    
    @property
    def object_front(self):
        return (self.obstacle_count[2] >= 5)
        # return (self.distances[2] < 10 and self.distances[2] > 0)
        # when object detected within 10cm, object detected
    
    @property
    def object_back(self):
        return (self.obstacle_count[3] >= 5)
        # return (self.distances[3] < 10 and self.distances[3] > 0)
        # when object detected within 10cm, object detected

    @property
    def check_obstacle(self):
        obstacle = []
        for i in range(1,5):
            if self.distances[i] < 10 and self.distances[i] > 0:
                obstacle.append(i)
        return obstacle


ultrasound = Ultrasound()
while True:
    ultrasound.receive_distances()
    distances = ultrasound.get_distances()  
    print("Distances:", distances)
    print("Obstacles at directions: {}".format(ultrasound.check_obstacle))
    time.sleep(0.1)  

# # Example usage:
# if __name__ == "__main__":
#     ultrasound = Ultrasound()  # Create an Ultrasound object
#     while True:
#         distances = ultrasound.receive_distances()
#         distances = ultrasound.get_distances()  
#         print("Distances:", distances)
#         time.sleep(0.1)  


from RosmasterBoard import Rosmaster
import time

bot = Rosmaster()
bot.create_receive_threading()

print(time.time())

def measure_stationary_yaw_drift_rate():
    start = time.time()
    roll, pitch, yaw = bot.get_imu_attitude_data()
    end = time.time()
    
    while end - start < 5:
        end = time.time()
        roll_end, pitch_end, yaw_end = bot.get_imu_attitude_data()
    
    yaw_rate = (yaw_end - yaw)/(end-start)
    return yaw_rate

print("Start Measure")
yaw_rate = measure_stationary_yaw_drift_rate()
print("Yaw Rate: {}".format(yaw_rate))


# while True:
#     roll, pitch, yaw = bot.get_imu_attitude_data()
#     print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

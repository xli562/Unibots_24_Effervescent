from RosmasterBoard import Rosmaster
import time

bot = Rosmaster()
bot.create_receive_threading()

imu_start = time.time()
roll_0, pitch_0, yaw_0 = bot.get_imu_attitude_data()


def measure_stationary_yaw_drift_rate():
    start = time.time()
    roll, pitch, yaw = bot.get_imu_attitude_data()
    end = time.time()
    
    while end - start < 5:
        end = time.time()
        roll_end, pitch_end, yaw_end = bot.get_imu_attitude_data()
    
    yaw_rate = (yaw_end - yaw)/(end-start)
    return yaw_rate


def get_imu_after_calibration(yaw_rate):
    roll, pitch, yaw = bot.get_imu_attitude_data()
    yaw = yaw - (time.time() - imu_start)*yaw_rate
    return yaw




print("Start Measure")
yaw_rate = measure_stationary_yaw_drift_rate()
print("Yaw Rate: {}".format(yaw_rate))

while True:
    real_yaw = get_imu_after_calibration(yaw_rate)
    print("Yaw: {}".format(real_yaw))
# while True:
#     roll, pitch, yaw = bot.get_imu_attitude_data()
#     print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

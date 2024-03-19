from RosmasterBoard import Rosmaster
import time
# import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

bot = Rosmaster()
bot.create_receive_threading()

imu_start = time.time()
roll_0, pitch_0, yaw_0 = bot.get_imu_attitude_data()

yaws=[]
times=[]


def measure_stationary_yaw_drift_rate(duration):
    yaws=[]
    times=[]

    start = time.time()
    roll, pitch, yaw = bot.get_imu_attitude_data()
    end = time.time()
    
    while end - start < duration:
        end = time.time()
        roll_end, pitch_end, yaw_end = bot.get_imu_attitude_data()
        times.append(end-start)
        yaws.append(yaw_end)

    plt.plot(times,yaws)
    plt.savefig('plt_no_offset.png')
    
    yaw_rate = (yaw_end - yaw)/(end-start)
    return yaw_rate


def get_imu_after_calibration(yaw_rate, duration):

    yaws=[]
    times=[]

    start = time.time()
    roll, pitch, yaw = bot.get_imu_attitude_data()
    yaw = yaw - (time.time() - imu_start)*yaw_rate
    end = time.time()
    
    while end - start < duration:
        end = time.time()
        roll_end, pitch_end, yaw_end = bot.get_imu_attitude_data()
        yaw_end = yaw_end - (time.time() - imu_start)*yaw_rate
        times.append(end-start)
        yaws.append(yaw_end)

    plt.plot(times,yaws)
    plt.savefig('plt_with_offset.png')
    




print("Start Measure")
yaw_rate = measure_stationary_yaw_drift_rate(20)
print('Finish measuring stationary drift')
get_imu_after_calibration(yaw_rate,20)
print('Finish')
# yaw_rate = measure_stationary_yaw_drift_rate()
# print("Yaw Rate: {}".format(yaw_rate))

# while True:
#     real_yaw = get_imu_after_calibration(yaw_rate)
#     print("Yaw: {}".format(real_yaw))
#     time.sleep(0.01)
# # while True:
# #     roll, pitch, yaw = bot.get_imu_attitude_data()
# #     print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

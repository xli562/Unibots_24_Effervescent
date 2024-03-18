from RosmasterBoard import Rosmaster

bot = Rosmaster()
while True:
    bot.get_imu_attitude_data()
    print("Roll: {}, Pitch: {}, Yaw: {}")

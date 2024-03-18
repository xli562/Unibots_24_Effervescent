from RosmasterBoard import Rosmaster

bot = Rosmaster()
while True:
    roll, pitch, yaw = bot.get_imu_attitude_data()
    print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

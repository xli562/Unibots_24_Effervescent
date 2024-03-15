#!/usr/bin/env python3
#coding=utf-8
import time
from Rosmaster_Lib import Rosmaster

# 创建Rosmaster对象 bot Create the Rosmaster object bot
bot = Rosmaster()

# 启动接收数据，只能启动一次，所有读取数据的功能都是基于此方法
# Start to receive data, can only start once, all read data function is based on this method
bot.create_receive_threading()

# 开启自动发送数据
# enable=True，底层扩展板会每隔40毫秒发送一次数据。enable=False，则不发送。
# forever=True永久保存，=False临时作用。
# Enable automatic data sending
# If enable=True, the underlying expansion module sends data every 40 milliseconds.  If enable=False, the port is not sent.
# Forever =True for permanent, =False for temporary
enable = True
bot.set_auto_report_state(enable, forever=False)

speed_x = 5
speed_y = 0
speed_z = 5


# 关闭自动发送数据
# enable=True，底层扩展板会每隔40毫秒发送一次数据。enable=False，则不发送。
# forever=True永久保存，=False临时作用。
# Disable automatic data sending
# If enable=True, the underlying expansion module sends data every 40 milliseconds.  If enable=False, the port is not sent.
# Forever =True for permanent, =False for temporary
enable = False
bot.set_auto_report_state(enable, forever=False)

# 清除单片机自动发送过来的缓存数据 Clear the cache data automatically sent by the MCU
bot.clear_auto_report_data()

# 控制电机运动 Control motor movement
bot.set_car_motion(speed_x, speed_y, speed_z)

# 获取小车线速度和角速度数据
# Obtain the linear velocity and angular velocity data of the car
try:
    while True:
        V_x, V_y, V_z = bot.get_motion_data()
        print("speed:", V_x, V_y, V_z, end='')
        print('encoder',bot.get_motor_encoder())
        print(bot.ge)
        # print(b'bot.FUNC_REPORT_IMU_RAW'wen)
        bot.clear_auto_report_data()
        time.sleep(.1)

except KeyboardInterrupt:
    pass


# 恢复出厂配置 Restoring factory Settings
bot.reset_flash_value()

# 程序结束后请删除对象，避免在其他程序中使用Rosmaster库造成冲突
# After the program is complete, delete the object to avoid conflicts caused by using the library in other programs
del bot
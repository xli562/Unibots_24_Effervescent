from RosmasterBoard import Rosmaster
import time

# import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

bot = Rosmaster()
bot.create_receive_threading()
bot.set_auto_report_state(True)

imu_start = time.time()
roll_0, pitch_0, yaw_0 = bot.get_imu_attitude_data()

yaws=[]
times=[]









class Chassis:
    def __init__(self) -> None:
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.yaw_tolerance = 1
        self.kP = 1
        self.kI = 0
        self.kD = 0
        self.yaw_rate = 0
        self.imu_global_start = bot.get_imu_attitude_data()[2]
    

    def measure_stationary_yaw_drift_rate(self, duration, plot=False):
        yaws=[]
        times=[]

        start = time.time()
        roll, pitch, yaw = bot.get_imu_attitude_data()
        end = time.time()
        
        while end - start < duration:
            end = time.time()
            roll_end, pitch_end, yaw_end = bot.get_imu_attitude_data()
            if plot:
                times.append(end-start)
                yaws.append(yaw_end)

        if plot:
            plt.plot(times,yaws)
            plt.savefig('plt_no_offset.png')
        
        yaw_rate = (yaw_end - yaw)/(end-start)

        self.yaw_rate = yaw_rate

        return yaw_rate

    def get_imu_after_calibration(yaw_rate, duration, plot=False):

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

            if plot:
                times.append(end-start)
                yaws.append(yaw_end)
        if plot:
            plt.plot(times,yaws)
            plt.savefig('plt_with_offset.png')
    
    def get_yaw_calibrated(self):
        _, _, yaw = bot.get_imu_attitude_data()
        yaw = yaw - (time.time() - self.imu_global_start)*self.yaw_rate
        return yaw

    def straight_forward(self):
        _, _, yaw_start_straight = self.get_yaw_calibrated()
        while True:
            try:
                _, _, yaw = self.get_yaw_calibrated()
                error = yaw - yaw_start_straight 
                self.vx = 2
                self.vy = 0
                self.xz = self.kP * error
                print("Error: {}, Vz: {}".format(error, self.vz))
            
                bot.set_car_motion(self.vx, self.xy, self.xz)
            except KeyboardInterrupt:
                break
    
    def right(self):
        _, _, yaw_start_straight = self.get_yaw_calibrated()
        while True:
            try:
                _, _, yaw = self.get_yaw_calibrated()
                error = yaw - yaw_start_straight 
                self.vx = 0
                self.vy = 2
                self.xz = self.kP * error
            
                bot.set_car_motion(self.vx, self.xy, self.xz)
            except KeyboardInterrupt:
                break
    
chassis = Chassis()

print("Start Measure")
yaw_rate = chassis.measure_stationary_yaw_drift_rate(20)
print("Yaw Rate: {}".format(yaw_rate))

# chassis.straight_forward()
# print('Finish')

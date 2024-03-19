from RosmasterBoard import Rosmaster
import time
from simple_pid import PID

# import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

bot = Rosmaster()
bot.create_receive_threading()
bot.set_auto_report_state(True)

# imu_start = time.time()
# roll_0, pitch_0, yaw_0 = bot.get_imu_attitude_data()

yaws=[]
times=[]




class Chassis:
    def __init__(self) -> None:
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.yaw_tolerance = 1
        #self.kP_staright = 1
        #self.kP_right = 0.1 #0.5 too oscillatory
        #self.kD = 0
        self.yaw_rate = 0
        self.time_global_start = time.time()
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

    # Not needed
    def get_imu_after_calibration(self, yaw_rate, duration, plot=False):

        yaws=[]
        times=[]

        start = time.time()
        roll, pitch, yaw = bot.get_imu_attitude_data()
        yaw = yaw - (time.time() - self.time_global_start)*yaw_rate
        end = time.time()
        
        while end - start < duration:
            end = time.time()
            roll_end, pitch_end, yaw_end = bot.get_imu_attitude_data()
            yaw_end = yaw_end - (time.time() - self.time_global_start)*yaw_rate

            if plot:
                times.append(end-start)
                yaws.append(yaw_end)
        if plot:
            plt.plot(times,yaws)
            plt.savefig('plt_with_offset.png')
    
    def get_yaw_calibrated(self):
        t = time.time()
        _, _, yaw = bot.get_imu_attitude_data()
        # print('Yaw Before Calibration: {}'.format(yaw))
        yaw = yaw - (t - self.time_global_start)*self.yaw_rate
        # print('Yaw After Calibration: {}'.format(yaw))
        return yaw

    def straight_forward(self):
        yaw_start = self.get_yaw_calibrated()
        pid_straight = PID(1,0,0, setpoint=yaw_start)
        while True:
            try:
                yaw = self.get_yaw_calibrated()
                control = pid_straight(yaw)
                error = yaw_start - yaw 
                self.vx = 1
                self.vy = 0
                self.vz = max(-10, min(control, 10))
                #self.vz = max(-10, min(self.kP_staright * error, 10))
                print("Error: {}, Vz: {}".format(error, self.vz))
                bot.set_car_motion(self.vx, self.vy, self.vz)
                time.sleep(0.1)
            except KeyboardInterrupt:
                bot.set_car_motion(0, 0, 0)
                break
    
    def right(self):
        yaw_start = self.get_yaw_calibrated()
        pid_right = PID(0.05, 0.1, 0, setpoint=yaw_start)
        while True:
            try:
                yaw = self.get_yaw_calibrated()
                control = pid_right(yaw)
                error = yaw - yaw_start
                self.vx = 0
                self.vy = 1
                self.vz = max(-10, min(control, 10))
                #self.vz = max(-10, min(self.kP_right * error, 10))
                print("Error: {}, Vz: {}".format(error, self.vz))
                bot.set_car_motion(self.vx, self.vy, self.vz)
                time.sleep(0.1)
            except KeyboardInterrupt:
                bot.set_car_motion(0, 0, 0)
                break
    

print('Program Start with a sleep of 15 seconds')
time.sleep(15)
bot.set_beep(100)
chassis = Chassis()
print("Start Measure")
yaw_rate = chassis.measure_stationary_yaw_drift_rate(15)
bot.set_beep(100)
print("Yaw Rate: {}".format(yaw_rate))
print('IMU gloabl start: {}'.format(chassis.imu_global_start))

#chassis.straight_forward()
chassis.right()
time.sleep(0.1)

# while True:
    # yaw = chassis.get_yaw_calibrated()
    # print('yaw: {}'.format(yaw))
    # error = chassis.imu_global_start - yaw
    # print('Error: {}'.format(error))

# chassis.straight_forward()

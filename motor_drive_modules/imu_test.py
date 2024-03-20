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

    def action(self, controller, yaw_start):
            yaw = self.get_yaw_calibrated()
            control = controller(yaw)
            error = yaw_start - yaw 
            self.vz = max(-10, min(control, 10))
            print("Error: {}, Control: {}, Vz: {}".format(error, control, self.vz))
            bot.set_car_motion(self.vx, self.vy, self.vz)
            time.sleep(0.1)

    def forward(self, duration = None):
        yaw_start = self.get_yaw_calibrated()
        pid_forward = PID(0.5,0,0.1, setpoint=yaw_start)
        self.vx = 1
        self.vy = 0
        if duration == None:
            while True:
                try:
                    self.action(pid_forward, yaw_start)
                except KeyboardInterrupt:
                    bot.set_car_motion(0, 0, 0)
                    break
        else:
            start = time.time()
            end = time.time()
            while end - start < duration:
                end = time.time()
                self.action(pid_forward, yaw_start)
            bot.set_car_motion(0, 0, 0)
            
        
    def backward(self, duration = None):
        yaw_start = self.get_yaw_calibrated()
        pid_backward = PID(0.5,0,0.1, setpoint=yaw_start)
        self.vx = -1
        self.vy = 0
        if duration == None:
            while True:
                try:
                    self.action(pid_backward, yaw_start)
                except KeyboardInterrupt:
                    bot.set_car_motion(0, 0, 0)
                    break
        else:
            start = time.time()
            end = time.time()
            while end - start < duration:
                end = time.time()
                self.action(pid_backward, yaw_start)
            bot.set_car_motion(0, 0, 0)
    
    def right(self, duration = None):
        yaw_start = self.get_yaw_calibrated()
        pid_right = PID(0.065, 0.1, 0.1, setpoint=yaw_start)
        self.vx = 0
        self.vy = 1
        if duration == None:
            while True:
                try:
                    self.action(pid_right, yaw_start)
                except KeyboardInterrupt:
                    bot.set_car_motion(0, 0, 0)
                    break
        else:
            start = time.time()
            end = time.time()
            while end - start < duration:
                end = time.time()
                self.action(pid_right, yaw_start)
            bot.set_car_motion(0, 0, 0)
        
    def left(self, duration = None):
        yaw_start = self.get_yaw_calibrated()
        pid_left = PID(0.065, 0.1, 0.1, setpoint=yaw_start)
        self.vx = 0
        self.vy = -1
        if duration == None:
            while True:
                try:
                    self.action(pid_left, yaw_start)
                except KeyboardInterrupt:
                    bot.set_car_motion(0, 0, 0)
                    break
        else:
            start = time.time()
            end = time.time()
            while end - start < duration:
                end = time.time()
                self.action(pid_left, yaw_start)
            bot.set_car_motion(0, 0, 0)
    
    def turn(self, angle): # -ve for right, +ve for left
        yaw_start = self.get_yaw_calibrated()
        pid_turn = PID(0.07, 0, 0.03, setpoint = yaw_start + angle)
        self.vx = 0
        self.vy = 0
        yaw = yaw_start
        previous_yaw = yaw_start
        while abs(yaw_start + angle - yaw) > 1:
            try:
                yaw = self.get_yaw_calibrated()
                if (yaw-previous_yaw) > 300:
                    yaw -= 360
                elif (yaw-previous_yaw) < -300:
                    yaw += 360
                previous_yaw = yaw
                error = yaw_start + angle - yaw 
                control = pid_turn(yaw)
                self.vz = max(-10, min(control, 10))
                print("Error: {}, Control: {}, Vz: {}".format(error, control, self.vz))
                bot.set_car_motion(self.vx, self.vy, self.vz)
                time.sleep(0.1)
            except KeyboardInterrupt:
                bot.set_car_motion(0, 0, 0)
                break
            
        bot.set_car_motion(0, 0, 0)

        
    
    def reset(self):
        bot.set_car_motion(0, 0 ,0)
    

print('Program Start with a sleep of 15 seconds')
time.sleep(15)
bot.set_beep(100)
chassis = Chassis()
print("Start Measure")
yaw_rate = chassis.measure_stationary_yaw_drift_rate(15)
bot.set_beep(100)
print("Yaw Rate: {}".format(yaw_rate))
print('IMU gloabl start: {}'.format(chassis.imu_global_start))

'''
chassis.forward(2)
time.sleep(0.2)
chassis.right(2)
time.sleep(0.2)
chassis.backward(2)
time.sleep(0.2)
chassis.left(2)
time.sleep(0.1)
'''
# while True:
#     print(chassis.get_yaw_calibrated())
#     time.sleep(0.1)
chassis.turn(-90)
time.sleep(0.2)
chassis.forward(1)
time.sleep(0.2)
chassis.turn(-90)
time.sleep(0.2)
chassis.forward(1)
time.sleep(0.2)
chassis.turn(-90)
time.sleep(0.2)
chassis.forward(1)
time.sleep(0.2)
chassis.turn(-90)
time.sleep(0.2)
chassis.forward(1)
time.sleep(0.2)
chassis.turn(-90)
time.sleep(0.2)
chassis.forward(1)
time.sleep(0.2)



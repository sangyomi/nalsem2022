from termios import N_MOUSE
import numpy as np
import nalsem_imu
import nalsem_motor
#import nalsem_neo
import time
from math import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
import atexit

from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

def handle_exit():
    time.sleep(1)
    M.__del__()

atexit.register(handle_exit) 

time.sleep(0.03)

def Imu_set(base_heading):

    N = nalsem_imu.find_heading()
    print("N:", N)

    M = base_heading - N

    if M > 180:
        M = M - 360
    
    elif M < - 180:
        M = M + 360
    
    return (M)

def L_speed_set(L_range):

    D_M = 1
    D_m = 0.1

    TF = [(L_range>D_m)*(L_range<D_M)]

    ID = np.where((L_range>D_m)*(L_range<D_M))

    if sum(np.where((L_range>D_m)*(L_range<D_M), True, False)) == 0:

        return 0

    for i in TF:

        VL = ((D_M-L_range[i])*ID)

        VL = VL/4       #bojung

    return (np.sum(VL))

def R_speed_set(R_range):

    D_M = 1
    D_m = 0.1

    TF = [(R_range>D_m)*(R_range<D_M)]

    ID = np.where((R_range>D_m)*(R_range<D_M))

    ID_R = (ID[0]-269)

    if sum(np.where((R_range>D_m)*(R_range<D_M), True, False)) == 0:

        return 0

    for i in TF:

        VL = ((D_M-R_range[i])*ID_R)
        VL = VL/4
        #bojung

    return (np.sum(VL))


time.sleep(1)
M = nalsem_motor.Motor()
time.sleep(1)

class Nalsem_Lidar(Node):

    def __init__(self):
        super().__init__('nalsem_lidar')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.LaserScan_callback,
            10)
        self.subscription # prevent unused variable warning

    def LaserScan_callback(self, msg):

        base_heading = 337

        L_range = np.array(msg.ranges[0:270])

        L_speed = L_speed_set(L_range)

        R_range = np.array(msg.ranges[270:540])

        R_speed = R_speed_set(R_range)

        print("L_speed_F:",L_speed*0.03)
        print("R_speed_F:",R_speed*-0.03)
        print("L_speed:",L_speed)
        print("R_speed:",R_speed)

        #if Bo_imu > 1200 or Bo_imu < -1200:
        #   if Bo_imu >= 0:
        #       M.motor_move(140, 70)
        #       time.sleep(1)
        #   elif Bo_imu < 0:
        #       M.motor_move(70, 140) 
        #       time.sleep(1)
        # if L_speed > 12000 and -R_speed > 12000 :
        #     print("corner")
            
        #     if L_speed >= -R_speed:
        #         M.motor_move(180,180)
        #         time.sleep(0.6)
        #         M.motor_move(180,30)
        #         time.sleep(4.0)
                               
        #     elif L_speed < -R_speed:
        #         M.motor_move(180,180)
        #         time.sleep(0.6)
        #         M.motor_move(30,180)
        #         time.sleep(4.0)

    #    if L_speed + -R_speed > 28000:
    #        print("back")

    #        if L_speed >= -R_speed:
    #            M.motor_move(150, 150)
    #            time.sleep(0.6)
    #            M.motor_move(140, 30)
    #            time.sleep(1.2)

    #       elif L_speed < -R_speed:
    #            M.motor_move(150, 150)
    #            time.sleep(0.6)
    #            M.motor_move(30, 140)
    #            time.sleep(1.2) 

        if L_speed < 1500 and -R_speed < 1500:
            U = Imu_set(base_heading)
            print("base_heading",base_heading)
            print("U:",U)
            if U < 2 and U > -2:
                M.motor_move(0,0)
            elif U < 60 and U > -60:
                if U < 0:
                    M.motor_move(120 - U*1, 7 + U*0.1)
                elif U > 0:
                    M.motor_move(7 - U*0.1 , 120 + U*1)
            else:
                if U < 0:
                    M.motor_move(180, 0)
                elif U > 0:
                    M.motor_move(0, 180)

        else:
            if L_speed >= -R_speed:
                print("Left")
                if L_speed*0.012 <= 20: 
                    M.motor_move(150 + L_speed*0.012, 20 - L_speed*0.012)
                else:
                    M.motor_move(180, 0)
                    
            elif L_speed < -R_speed:
                print("Right")
                if R_speed*-0.012 <= 20:
                    M.motor_move(20 - R_speed*-0.012, 150 + R_speed*-0.012)
                else:
                    M.motor_move(0, 180)

def main(args=None):

    rclpy.init(args=args)

    nalsem_lidar = Nalsem_Lidar()

    rclpy.spin(nalsem_lidar)

    nalsem_lidar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

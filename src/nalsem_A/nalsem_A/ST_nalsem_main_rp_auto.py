from termios import N_MOUSE
import numpy as np
#import nalsem_imu
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
    print("M", M)

    if M > 180:
        M = M - 360
    
    elif M < - 180:
        M = M + 360
    
    return (M)

def L_speed_set(L_range):

    D_M = 1.6
    D_m = 0.1

    TF = [(L_range>D_m)*(L_range<D_M)]

    ID = np.where((L_range>D_m)*(L_range<D_M))

    if sum(np.where((L_range>D_m)*(L_range<D_M), True, False)) == 0:

        return 0

    for i in TF:

        VL = ((D_M-L_range[i])*ID)

        VL = VL/5.5
        #bojung

    return (np.sum(VL))

def R_speed_set(R_range):

    D_M = 1.6
    D_m = 0.1

    TF = [(R_range>D_m)*(R_range<D_M)]

    ID = np.where((R_range>D_m)*(R_range<D_M))

    ID_R = (ID[0]-499)

    if sum(np.where((R_range>D_m)*(R_range<D_M), True, False)) == 0:

        return 0

    for i in TF:

        VL = ((D_M-R_range[i])*ID_R)

        VL = VL/5.5
        #bojung

    return (np.sum(VL))


time.sleep(1)
M = nalsem_motor.Motor()
time.sleep(1)

#for i in range(0,50):
    
#    base_heading = nalsem_imu.find_heading()

class Nalsem_Lidar(Node):

    def __init__(self):
        super().__init__('nalsem_lidar')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.LaserScan_callback,
            1)
        self.subscription # prevent unused variable warning

    def LaserScan_callback(self, msg):

        #U = Imu_set(base_heading)

        #print(U)

        L_range = np.array(msg.ranges[0:500])

        L_speed = L_speed_set(L_range)

        R_range = np.array(msg.ranges[500:1000])

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
        if L_speed > 12000 and -R_speed > 12000 :
            print("corner")
            
            if L_speed >= -R_speed:
                M.motor_move(180,180)
                time.sleep(0.6)
                M.motor_move(180,30)
                time.sleep(4.0)
                               
            elif L_speed < -R_speed:
                M.motor_move(180,180)
                time.sleep(0.6)
                M.motor_move(30,180)
                time.sleep(4.0)

        elif L_speed + -R_speed > 21500:
            print("back")

            if L_speed >= -R_speed:
                M.motor_move(180, 180)
                time.sleep(0.6)
                M.motor_move(150, 40)
                time.sleep(1.2)

            elif L_speed < -R_speed:
                M.motor_move(180, 180)
                time.sleep(0.6)
                M.motor_move(40, 150)
                time.sleep(1.2) 

        elif L_speed < 1000 and -R_speed < 1000:
         #  if U > 25:
         #      U = U*0.3
         #      M.motor_move(85 - U, 120 + U)
         #  elif U < -25:
         #       U = U*0.3
         #      M.motor_move(85 - U, 120 + U)
         #   else:
         #       M.motor_move(75,75)
            M.motor_move(77,77)
        else:
            if L_speed >= -R_speed:
                print("Left")

                if L_speed*0.05 <= 50: 
                    M.motor_move(120 + L_speed*0.05, 80 - L_speed*0.05)
                else:
                    M.motor_move(170, 30)
                    
            elif L_speed < -R_speed:
                print("Right")

                if R_speed*-0.05 <= 50:
                    M.motor_move(80 - R_speed*-0.05, 120 + R_speed*-0.05)
                else:
                    M.motor_move(30, 170)

def main(args=None):

    rclpy.init(args=args)

    nalsem_lidar = Nalsem_Lidar()

    rclpy.spin(nalsem_lidar)

    nalsem_lidar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

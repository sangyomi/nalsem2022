import numpy as np
import nalsem_imu
#import nalsem_gps
import nalsem_motor
import time
from math import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
import atexit

def handle_exit():
    time.sleep(1)
    M.__del__()

atexit.register(handle_exit) 

time.sleep(1)
M = nalsem_motor.Motor()
time.sleep(1)

base_heading = nalsem_imu.find_heading()

print("base_heading:", base_heading)

a = list()
b = list()

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

        time.sleep(0.02)

        p_heading = nalsem_imu.find_heading()

        imu_move = base_heading - p_heading

        print(p_heading)        
        print("base_heading:", imu_move)

        M.moter_move(70,110) #i_V
        
        L = np.array(msg.ranges[20:40])
        R = np.array(msg.ranges[0:20])    

        L_sum = sum(np.where(L < 1, True, False))
        R_sum = sum(np.where(R < 1, True, False))

        # if L_sum >= R_sum:
        #     if sum(np.where(L < 1, True, False)) > 10:
        #         print(sum(np.where(L < 1, True, False)))
        #         M.move_right(65, 0.25)
        #         print("L.lidar_move_turn_left")
        #     elif imu_move > 15 : 
        #         M.move_right(65, 0.25)
        #         print("imu_move_turn_left")
        #     elif imu_move < -15 : 
        #         M.move_left(65, 0.25)
        #         print("imu_move_turn_right")
                
        # elif L_sum < R_sum:    
        #     if sum(np.where(R < 1, True, False)) > 10:
        #         print(sum(np.where(R < 1, True, False)))
        #         M.move_left(65, 0.25)
        #         print("R.lidar_move_turn_right")
        #     elif imu_move > 15 : 
        #         M.move_right(65, 0.25)
        #         print("imu_move_turn_left")
        #     elif imu_move < -15 : 
        #         M.move_left(65, 0.25)
        #         print("imu_move_turn_righ")


def main(args=None):

    rclpy.init(args=args)

    nalsem_lidar = Nalsem_Lidar()

    rclpy.spin(nalsem_lidar)

    nalsem_lidar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
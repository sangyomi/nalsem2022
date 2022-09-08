import numpy as np
import time
from math import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy



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

        L_range = np.array(msg.ranges[0:10])

        print(L_range)

        R_range = np.array(msg.ranges[360:370])

        print(R_range)
                
        R_range = np.array(msg.ranges[720:730])

        print(R_range)
        
        R_range = np.array(msg.ranges[1080:1090])

        print(R_range)

        R_range = np.array(msg.ranges[1440:1450])

        print(R_range)

        print('=======================================')




def main(args=None):

    rclpy.init(args=args)

    nalsem_lidar = Nalsem_Lidar()

    rclpy.spin(nalsem_lidar)

    nalsem_lidar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import numpy as np
import nalsem_imu
import nalsem_motor
import nalsem_neo
import nalsem_gps
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

ser = serial.Serial(port = "/dev/ttyACM0", baudrate = 38400, timeout = 0.1)	
datName = 'GNGLL'

def GPSabsorption(data):
	data = data.decode("utf-8")
	gps_data = list()
	
	if data[1:6] == datName:
		spliteddata = data.split(",")
		data2 = float(spliteddata[1]) 
		data3 = float(spliteddata[3]) 
		return data2, data3
	return "data invalid"
##GPS_setup

def ConvertDecimalDegreesToRadians(deg):
	return (deg * PI / 180)

def ConvertRadiansToDecimalDegrees(rad):
	return (rad * 180 / PI)

def GetBearing(lat1,lon1,lat2,lon2):
	lat1_rad = ConvertDecimalDegreesToRadians(lat1)
	lat2_rad = ConvertDecimalDegreesToRadians(lat2)
	lon_diff_rad = ConvertDecimalDegreesToRadians(lon2-lon1)
	y = sin(lon_diff_rad) * cos(lat2_rad)
	x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(lon_diff_rad)
	return (ConvertRadiansToDecimalDegrees(atan2(y,x)) + 360) % 360

def GetDistance(lat1,lon1,lat2,lon2):
    theta = lon1 - lon2
    dist = sin(ConvertDecimalDegreesToRadians(lat1)) * sin(ConvertDecimalDegreesToRadians(lat2)) + cos(ConvertDecimalDegreesToRadians(lat1)) * cos(ConvertDecimalDegreesToRadians(lat2)) * cos(ConvertDecimalDegreesToRadians(theta))
    dist = acos(dist)
    dist = ConvertRadiansToDecimalDegrees(dist)
    dist = dist * 60 *1.1515
    dist = dist * 1.609344 * 1000
    return dist

def handle_exit():
    time.sleep(1)
    M.__del__()

atexit.register(handle_exit) 

time.sleep(0.02)

def Imu_set(base_heading):

    time.sleep(0.02)

    Bo = base_heading - 180 
    
    if nalsem_imu.find_heading() <= Bo :

        now_heading = nalsem_imu.find_heading() - Bo + 360

    else:

        now_heading = nalsem_imu.find_heading() - Bo
    
    return (now_heading)

def L_speed_set(L_range):

    D_M = 1.25
    D_m = 0.1

    TF = [(L_range>D_m)*(L_range<D_M)]

    ID = np.where((L_range>D_m)*(L_range<D_M))

    if sum(np.where((L_range>D_m)*(L_range<D_M), True, False)) == 0:

        return 0

    for i in TF:

        VL = ((D_M-L_range[i])*ID)

        VL = VL/2 #bojung

    return (np.sum(VL))

def R_speed_set(R_range):

    D_M = 1.25
    D_m = 0.1

    TF = [(R_range>D_m)*(R_range<D_M)]

    ID = np.where((R_range>D_m)*(R_range<D_M))

    ID_R = (ID[0]-159)

    if sum(np.where((R_range>D_m)*(R_range<D_M), True, False)) == 0:

        return 0

    for i in TF:

        VL = ((D_M-R_range[i])*ID_R)

        VL = VL/2 #bojung

    return (np.sum(VL))


time.sleep(1)
M = nalsem_motor.Motor()
time.sleep(1)

Goal_x = 35.1392528
Goal_y = 129.0475794

for i in range(0,5):
    
    base_heading = nalsem_imu.find_heading()

class Nalsem_Lidar(Node):

    def __init__(self):
        super().__init__('nalsem_lidar')
        qos_profile = QoSProfile(depth = 10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        self.subscription = self.create_subscription(
            LaserScan,
            '/nalsem_A/scan',
            self.LaserScan_callback,
            qos_profile)
        self.subscription # prevent unused variable warning

    def LaserScan_callback(self, msg):

        time.sleep(0.02)

        now_heading = Imu_set(base_heading)

        Bo_imu = now_heading -180

        print("Bo_imu:", Bo_imu)

        L_range = np.array(msg.ranges[0:160])

        L_speed = L_speed_set(L_range)

        R_range = np.array(msg.ranges[160:320])

        R_speed = R_speed_set(R_range)

        print("L_speed_F:",L_speed*0.02)
        print("R_speed_F:",R_speed*-0.02)
        print("L_speed:",L_speed)
        print("R_speed:",R_speed)

        # if Bo_imu > 120 or Bo_imu < -120:

        #     if Bo_imu > 0:
        #         M.moter_move(85, 100)

        #     elif Bo_imu < 0:
        #         M.moter_move(100, 85)

        if L_speed + -R_speed > 5000:

            X = (L_speed + -R_speed)*0.01
            M.moter_move(130 , 130)

        elif L_speed >= -R_speed:

            if L_speed*0.015 <= 70: 
                M.moter_move(80,80-L_speed*0.015)
            else:
                M.moter_move(80,10)
                
        elif L_speed < -R_speed:

            if R_speed*-0.015 <= 70:
                M.moter_move(80-R_speed*-0.015,80)
            else:
                M.moter_move(10,80)

        data = nalsem_gps.GPSabsorption
        x = data[0]
        y = data[1]

        if GetDistance(x,y,Goal_x,Goal_y) < 2:
            nalsem_neo.Color


def main(args=None):

    rclpy.init(args=args)

    nalsem_lidar = Nalsem_Lidar()

    rclpy.spin(nalsem_lidar)

    nalsem_lidar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

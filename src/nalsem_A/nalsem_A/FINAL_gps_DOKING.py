from locale import D_T_FMT
import numpy as np
import nalsem_imu
import nalsem_motor
import nalsem_gps 
import nalsem_camera_blue
#import nalsem_camera_red
#import nalsem_camera_green
import serial
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

# def Imu_set(base_heading):

#     time.sleep(0.02)

#     Bo = base_heading - 180 
    
#     if nalsem_imu.find_heading() <= Bo :

#         now_heading = nalsem_imu.find_heading() - Bo + 360

#     else:

#         now_heading = nalsem_imu.find_heading() - Bo
    
#     return (now_heading)

def L_speed_set(L_range):

    D_M = 1.6
    D_m = 0.1

    TF = [(L_range>D_m)*(L_range<D_M)]

    ID = np.where((L_range>D_m)*(L_range<D_M))

    if sum(np.where((L_range>D_m)*(L_range<D_M), True, False)) == 0:

        return 0

    for i in TF:

        VL = ((D_M-L_range[i])*ID)

        VL = VL/3
        #bojung

    return (np.sum(VL))

def R_speed_set(R_range):

    D_M = 1.6
    D_m = 0.1

    TF = [(R_range>D_m)*(R_range<D_M)]

    ID = np.where((R_range>D_m)*(R_range<D_M))

    ID_R = (ID[0]-249)

    if sum(np.where((R_range>D_m)*(R_range<D_M), True, False)) == 0:

        return 0

    for i in TF:

        VL = ((D_M-R_range[i])*ID_R)

        VL = VL/3
        #bojung

    return (np.sum(VL))

def GPSabsorption(data):
    data = data.decode("utf-8")
    gps_data = list()

    if data[1:6] == datName:
        spliteddata = data.split(",")
        data2 = float(spliteddata[1]) 
        data3 = float(spliteddata[3]) 
        return data2, data3
    return "data invalid"

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

def Doking():
    M.motor_move(160,160)
    time.sleep(0.4)
    M.motor_move(70,70)
    time.sleep(0.2)
    while True:
        print("Doking")
        D = nalsem_camera_blue.camera()
        if D != "DEL":
            print(D)
            D = D - 480 
            M.motor_move(100 - D*0.12 , 100 + D*0.12) 
        elif D == "DEL" :
            M.motor_move(125, 85)

PI = 3.141592

ser = serial.Serial(port = "/dev/ttyACM0", baudrate =115200, timeout = 0.1)	
datName = 'GNGLL'

time.sleep(1)
M = nalsem_motor.Motor()
time.sleep(1)

for i in range(0,20):
    
    base_heading = nalsem_imu.find_heading()

    GPS_S = 0 

while GPS_S == 20:
    gps_data = nalsem_gps.GPSabsorption()
        
    if gps_data != "data invalid":
        print("gps on")
        x_head = gps_data[0] * 0.01                   
        y_head = gps_data[1] * 0.01
        print(x_head)
        print(y_head)
        GPS_S = GPS_S + 1

Goal_x = 35.0417629
Goal_y = 128.3473219

ang_head = GetBearing(x_head,y_head,Goal_x,Goal_y)


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

    #   now_heading = Imu_set(base_heading)

    #    Bo_imu = now_heading -180

    #    print("base_heading", base_heading)

    #    print("Bo_imu:", Bo_imu)

        L_range = np.array(msg.ranges[0:250])

        L_speed = L_speed_set(L_range)

        R_range = np.array(msg.ranges[250:500])

        R_speed = R_speed_set(R_range)

        Goal_x = 35.0417629
        Goal_y = 128.3473219

        GPS_S = 0         
        while GPS_S == 0:
            gps_data = nalsem_gps.GPSabsorption()
                
            if gps_data != "data invalid":
                print("gps on")
                x = gps_data[0] * 0.01                   
                y = gps_data[1] * 0.01
                print(x)
                print(y)
                GPS_S = GPS_S + 1

            else : 
                GPS_S = 0

        # gps_data = nalsem_gps.GPSabsorption()
            
        # if gps_data != "data invalid":
        #     print("gps on")
        #     x = gps_data[0] * 0.01                   
        #     y = gps_data[1] * 0.01
        #     print(x)
        #     print(y)

        ang = GetBearing(x,y,Goal_x,Goal_y) 

        gps_head = ang_head - ang


        print("L_speed_F:",L_speed)
        print("R_speed_F:",R_speed)
        print("L_speed:",L_speed)
        print("R_speed:",R_speed)

        #if Bo_imu > 1200 or Bo_imu < -1200:
        #   if Bo_imu >= 0:
        #       M.motor_move(140, 70)
        #       time.sleep(1)
        #   elif Bo_imu < 0:
        #       M.motor_move(70, 140) 
        #       time.sleep(1)
        if L_speed + -R_speed > 10000:
            print("back")
           # if K == 1:
           #     M.motor_move(150, 40)
           #     time.sleep(3)

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
            print("default")

            if gps_head > 30 :
                M.motor_move(130, 80)
                time.sleep(0.6)

            elif gps_head < 30 :
                M.motor_move(80, 130)
                time.sleep(0.6)                

            M.motor_move(77, 77)

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
from dis import dis
import time
from math import *
from matplotlib.pyplot import disconnect
import serial
import operator
from board import SCL, SDA
from busio import I2C
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan 
import Jetson.GPIO as GPIO
import numpy as np
import nalsem_motor
import nalsem_imu
import nalsem_gps 
import rclpy
from rclpy.node import Node
import atexit

def handle_exit():
    time.sleep(1)
    M.__del__()
atexit.register(handle_exit) 

ser = serial.Serial(port = "/dev/ttyACM0", baudrate = 115200, timeout = 0.1)	
datName = 'GNGLL'

PI = 3.141592

time.sleep(1)
M = nalsem_motor.Motor()
time.sleep(1)

Goal_x = [35.0416797, 35.0416971, 35.0417355, 35.0417692, 35.0416276]
Goal_y = [128.3473816, 128.3473222, 128.3473596, 128.3472933, 128.3473756]

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
			
def Goal(Goal_x, Goal_y, i):

    if i == 4:
        X = 65
        G = 10
    else:
        X = 30
        G = 0
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
    A = nalsem_imu.find_heading() 
    print("IMU",A)
    ang = GetBearing(x,y,Goal_x,Goal_y) - A
    dis = GetDistance(x,y,Goal_x,Goal_y) 
    print(ang)
    print(dis)
    
    if ang > 180:
        ang = ang - 360
        print("ang======",ang)
        move = -ang*0.8
        if move < 5 + G:
            M.motor_move(65-X,65-X)
        elif move < 50:
            M.motor_move(150 + move*0.6, 50 - move)
        else:
            M.motor_move(180, 25)

    elif ang < -180 :
        ang = ang + 360
        print("ang======",ang) 
        move = ang*0.8
        if move < 5 + G:
            M.motor_move(65-X, 65-X)
        elif move < 50:
            M.motor_move(50 - move , 150 + move*0.6)
        else: 
            M.motor_move(25, 180)
           
    else:    
        if ang < 0 :
            print("ang======",ang)
            move = -ang*0.8
            if move < 5 + G:
                M.motor_move(65-X, 65-X)
            elif move < 50:
                M.motor_move(150 + move*0.6 , 50 - move)
            else:
                M.motor_move(180, 25)
        elif ang > 0 : 
            print("ang======",ang)
            move = ang*0.8
            if move < 5 + G:
                M.motor_move(65-X, 65-X)
            elif move < 50:
                M.motor_move(50 - move, 150 + move*0.6)
            else:
                M.motor_move(25, 180)

    return dis

i = 0
while True:
    dis = Goal(Goal_x[i], Goal_y[i], i)
    if dis < 1.2 :
        i = i + 1
       # M.motor_move(160,160)
       # time.sleep(0.5)
       # M.motor_move(50,50)
       # time.sleep(0.3)
        print("+++####++++++++++++Goal point %d+++++++++++++####+++", i)
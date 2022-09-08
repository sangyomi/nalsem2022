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

def Goal_2():
	Goal_x = 35.139220202
	Goal_y = 129.047543898

	while True:	

		M.motor_move(95, 95)

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

		ang = GetBearing(x,y,Goal_x,Goal_y) - nalsem_imu.find_heading()
		dis = GetDistance(x,y,Goal_x,Goal_y) 
		print(GetBearing(x,y,Goal_x,Goal_y))
		print(nalsem_imu.find_heading())
		print(ang)
		print(dis)

		if ang < 0 :
			if -ang*0.7 < 70:
				M.moter_move(75, 75 + ang*0.7)
				time.sleep(0.2)
			else:
				M.moter_move(75, 10)
				time.sleep(0.2)
		elif ang > 0 : 
			if ang*0.7 < 70:
				M.moter_move(75 - ang*0.7, 83)
				time.sleep(0.2)
			else: 
				M.moter_move(10, 75)
				time.sleep(0.2)
		if dis < 0.8 :
			print("============Goal_3=============")
			M.moter_move(120, 120)
			time.sleep(4)
			

PI = 3.141592

time.sleep(1)
M = nalsem_motor.Motor()
time.sleep(1)

Goal_x = 35.1392596
Goal_y = 129.0475797

while True:	

	M.moter_move(75, 75)

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
#	if nalsem_imu.find_heading() >= 180:
#		A =	nalsem_imu.find_heading() -180
#	else:
#		A =	nalsem_imu.find_heading() + 180
	time.sleep(0.2)
	ang = GetBearing(x,y,Goal_x,Goal_y) - nalsem_imu.find_heading()
	dis = GetDistance(x,y,Goal_x,Goal_y) 
	print(GetBearing(x,y,Goal_x,Goal_y))
	print(nalsem_imu.find_heading())
	print(ang)
	print(dis)

	if ang < 0 :
		if -ang*0.7 < 70:
			M.moter_move(75, 75 + ang*0.7)
			time.sleep(0.2)
		else:
			M.moter_move(75, 10)
			time.sleep(0.2)
	elif ang > 0 : 
		if ang*0.7 < 70:
			M.moter_move(75 - ang*0.7, 75)
			time.sleep(0.2)
		else: 
			M.moter_move(10, 75)
			time.sleep(0.2)

	if dis < 1 :
		print("============Goal_2=============")
		Goal_2()

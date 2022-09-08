import time
from math import *
import serial
import operator
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA, frequency=800000)
bno = BNO08X_I2C(i2c)

bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

def find_heading():
    uni = 0
    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    sqw = quat_real*quat_real
    sqx = quat_i*quat_i
    sqy = quat_j*quat_j
    sqz = quat_k*quat_k
    uni = sqx + sqy + sqz + sqw
    test = quat_i*quat_j + quat_k*quat_real
    if test > 0.499*uni : 
        print("a")
        heading = 2 * atan2(quat_i,quat_real)
        return heading
	
    if test < -0.499*uni :
        print("b")
        heading = -2 * atan2(quat_i,quat_real)
        return heading
	
    heading = atan2(2*quat_j*quat_real-2*quat_i*quat_k , sqx - sqy - sqz + sqw)
    print("c")
    return heading  # heading in 360 clockwis

while True :
    yaw = find_heading()
    print(yaw)

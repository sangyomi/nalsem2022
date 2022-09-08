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
    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    norm = sqrt(quat_real * quat_real + quat_i * quat_i + quat_j * quat_j + quat_k * quat_k)
    quat_real = quat_real / norm
    quat_i = quat_i / norm
    quat_j = quat_j / norm
    quat_k = quat_k / norm

    ysqr = quat_j * quat_j

    t3 = +2.0 * (quat_real * quat_k + quat_i * quat_j)
    t4 = +1.0 - 2.0 * (ysqr + quat_k * quat_k)
    yaw_raw = atan2(t3, t4)
    yaw = yaw_raw * 180.0 / 3.141592
    if yaw > 0:
        yaw = 360 - yaw
    else:
        yaw = abs(yaw)
    return yaw  # heading in 360 clockwis





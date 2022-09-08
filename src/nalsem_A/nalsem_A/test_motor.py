
import numpy as np
from nalsem_motor import *
import time
from math import *
import atexit

def handle_exit():
    time.sleep(1)
    M.__del__()

atexit.register(handle_exit) 
print("A")
time.sleep(1)
M = Motor()
time.sleep(1)
print("B")
M.motor_move(0, 180)
time.sleep(10)
print("C")
M.motor_move(90,90)
time.sleep(10)
M.motor_move(180,0)
time.sleep(10)

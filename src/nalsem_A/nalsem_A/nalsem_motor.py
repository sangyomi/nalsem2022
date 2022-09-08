from adafruit_servokit import ServoKit 
import board     
import busio     
import time        
from board import SCL, SDA
from busio import I2C
import Jetson.GPIO as GPIO

class Motor:

    def __init__(self):
        print("Motor init")

        i2c_bus0 = (busio.I2C(board.SCL_1, board.SDA_1))

        self.servo_kit = ServoKit(channels=16, i2c=i2c_bus0)

        print("Motor set")

        self.servo_kit.servo[0].set_pulse_width_range(1100, 1900)
        self.servo_kit.servo[1].set_pulse_width_range(1100, 1900)

        self.servo_kit.servo[0].angle = 90
        self.servo_kit.servo[1].angle = 90

        time.sleep(1)        

        print("Motor start")

    def motor_move(self, speedleft: int, speedright: int):
    
        self.servo_kit.servo[0].angle = speedleft
        self.servo_kit.servo[1].angle = speedright

    def __del__(self):

        print("Motor del")

        self.servo_kit.servo[0].angle = 90
        self.servo_kit.servo[1].angle = 90
       
        del self.servo_kit

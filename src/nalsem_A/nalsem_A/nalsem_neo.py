from urllib import response
import serial
import threading
import time

port = '/dev/ttyACM1'
baudrate = 115200
ser = serial.Serial(port, baudrate, timeout=1)

#def Color():
 #   num = '1'
  #  num = num.encode('utf-8')
   # ser.write(num)
    #time.sleep(1)

#Color() 

#ser = serial.Serial('/dev/ttyACM1', 115200)
commend = input("G")
while True:
    ser.write(commend.encode())
    time.sleep(0.1)


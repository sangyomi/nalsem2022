import serial
import time

ser = serial.Serial(port = "/dev/ttyACM0", baudrate = 38400, timeout = 0.1)	
datName = 'GNGLL'

def GPSabsorption():
    data = ser.readline()
    data = data.decode("utf-8")
    gps_data = list()
            
    if data[1:6] == datName:
        spliteddata = data.split(",")
        data2 = float(spliteddata[1]) 
        data3 = float(spliteddata[3])
        print(data2)
        print(data3) 
        return data2, data3
    return "data invalid"

while True:
    
    GPS_S = 0 
            
    while GPS_S == 0:	
        gps_data = GPSabsorption()		
        if gps_data != "data invalid":
            print("gps on")
            x = gps_data[0] * 0.01                   
            y = gps_data[1] * 0.01
            print(x)
            print(y)
            GPS_S = GPS_S + 1        
        else : 
            GPS_S = 0 
    

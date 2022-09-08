
import serial

ser = serial.Serial(port = "/dev/ttyACM0", baudrate = 115200, timeout = 0.1)	
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

data = GPSabsorption()

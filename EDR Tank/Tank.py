import serial
import time
import struct

ser = serial.Serial(port='COM9', baudrate=57600)
print("connected to: " + ser.portstr)

a = 0;

while True:
    
    sendString = "%d %d" % (a,-a)

    ser.write(sendString.encode())
    print("Sent data")

    a += 1
    
    time.sleep (0.1)
    
    result = ser.readline()
    print("> " + result.decode('utf-8')) 
 
ser.close()
print("close")

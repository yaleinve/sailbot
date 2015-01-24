from serial import Serial
import time

ser = Serial("/dev/ttyO0", timeout = 2)
ser.open()
ser.flushInput()
ser.flushOutput()

while True:
    bytesToRead = ser.inWaiting()
    print(bytesToRead)
 

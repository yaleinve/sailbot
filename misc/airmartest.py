import serial
import pynmea2
import pdb

ser = serial.Serial('/dev/ttyUSB0', 4800, timeout=1)
line = ser.readline()
while True:
        line = ser.readline()
        try:
                msg = pynmea2.parse(line)
                if msg.sentence_type == 'MWV':
                        print msg.wind_angle
        except:
                "Error!"

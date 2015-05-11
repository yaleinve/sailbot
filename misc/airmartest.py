import serial
import pynmea2
import pdb
import string

ser = serial.Serial('/dev/ttyUSB1', 4800, timeout=1)
line = ser.readline()
while True:
        line = ser.readline()
        try:
                msg = pynmea2.parse(line)
                if msg.sentence_type == 'MWD':
                	print "True Wind (knots): " + str(msg).split(",")[5] + " True Wind (m/s): " + str(msg).split(",")[7]
                if msg.sentence_type == 'MWV':
                    print "Wind Speed: " + msg.wind_speed
                #if msg.sentence_type == 'GGA':
                #	print "lat: " + str(msg.latitude) + " long: " + str(msg.longitude)
                '''	if msg.sentence_type == 'XDR' and msg.type == 'A':
                    print str(msg).split(",")[6]'''

        except:
                "Error!"

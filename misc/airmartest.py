import serial
import pynmea2
import pdb
import string

ser = serial.Serial('/dev/ttyUSB0', 4800, timeout=1)
line = ser.readline()
while True:
        line = ser.readline()
        try:
                msg = pynmea2.parse(line)
                if msg.sentence_type == 'VTG':
                	print "COG: " + str(msg.true_track) + " SOG: " + str(msg.spd_over_grnd_kts)
                '''if msg.sentence_type == 'GGA':
                	print "lat: " + str(msg.latitude)
                	print "long: " + str(msg.longitude)
                	if msg.sentence_type == 'XDR' and msg.type == 'A':
                    print str(msg).split(",")[6]'''

        except:
                "Error!"

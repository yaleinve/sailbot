#!/usr/bin/env python
#A non-ros runner to test edison serial

import mraa
import random
import math
import serial
import pynmea2
import pdb
import time

class Airmar:
    def __init__(self):
        self.heading = 0
        self.amrRoll = 0
        self.truWndDir = 0
        self.truWndSpd = 0
        self.cog = 0
        self.lat = 0
        self.long = 0
        self.sog = 0
        self.wndUnits = "K"
        self.apWndDir = 0
        self.apWndSpd = 0

        # we need to change this port to whatever it's going to be for the
        # actual machine we run ROS on 
        
        #Turn pins 0 and 1 on to uart so we can actually use them
        mraa.Uart(0)
        #...but, mraa is a bitch, after they're turned on let's just use pyserial library instead ;)
        self.ser = serial.Serial('/dev/ttyMFD1', 4800, timeout=1)
        self.ser.readline()

    def __str__(self):
        return str(self.__dict__)

    def update_data(self, debug=False):
        '''
        Fetches the latest data off the serial wire from the Airmar
        debug -- if True, publish fake data, else, publish real data
        '''
        line = self.ser.readline()
        print "\n input line : \n" + str(line) + "\n"
        if "," in str(line):
            try:
                msg = pynmea2.parse(line)
                # self.lat = msg.latitude
                # self.long = msg.longitude
                if msg.sentence_type == 'MWV':
                    self.apWndSpd = msg.wind_speed if msg.wind_speed != None else self.apWndSpd
                    self.wndUnits = msg.wind_speed_units if msg.wind_speed_units != None else self.wndUnits
                    self.apWndDir = msg.wind_angle if msg.wind_angle != None else self.apWndDir
                elif msg.sentence_type == 'MWD':
                    self.truWndSpd = msg.wind_speed_knots if msg.wind_speed_knots != None else self.truWndSpd
                    self.truWndDir = msg.direction_true if msg.direction_true != None else self.truWndDir
                elif msg.sentence_type == 'HDT': # change to HDG for magnetic heading
                    self.heading = msg.heading if msg.heading != None else self.heading
                elif msg.sentence_type == 'VTG':
                    self.cog = msg.true_track if msg.true_track != None else self.cog # change to mag_track for magnetic cog
                    self.sog = msg.spd_over_grnd_kts if msg.spd_over_grnd_kts != None else self.sog# can also support kmph
                elif msg.sentence_type == 'GGA':
                    self.lat = msg.latitude if msg.latitude != None else self.lat
                    self.long = msg.longitude if msg.longitude != None else self.long
                elif msg.sentence_type == 'XDR' and msg.type == 'A':
                    self.amrRoll = float(str(msg).split(",")[6]) if str(msg).split(",")[6] != None else self.amrRoll 
                    # The pynmea2 library can't parse our XDR messages properly
                    # This basically grabs the roll value manually from the message (it's in a standard form)
                

            except Exception, e:
                print "[airmar] Error!"
                print str(e)
        print "Next value from airmar:" 
        print self

if __name__ == '__main__':
    try:
        am = Airmar()

        while True:
            am.update_data() # Get the latest data from the airmar
            time.sleep(1)
    except Exception as e:
        print e

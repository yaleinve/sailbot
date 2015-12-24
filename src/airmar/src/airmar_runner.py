#!/usr/bin/env python

#Updated by Eric Anderson 12/15 to adapt for Edison and recent arch changes,
#namely that we rely solely on airmar (and camera) for sensing - no navsat

import rospy
import random
import math
import serial
import pynmea2
import pdb
import time
import mraa

from airmar.msg import AirmarData

class Airmar:
    def __init__(self):
        #Set the publishing rate of the airmar to 2Hz for now
        self.pubInterval = 0.5      # Seonds between publishes

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

        #Initialize a publisher
        self.pub = rospy.Publisher('/airmar_data', AirmarData, queue_size = 10)
        
        #Use mraa to initialize edison pins 0 and 1 to serial
        mraa.Uart(0)
        #...but, after initialized, let's use the pyserial library
        self.ser = serial.Serial('/dev/ttyMFD1', 4800, timeout=1)
        self.ser.readline()

    def update_data(self, debug=False):
        '''
        Fetches the latest data off the serial wire from the Airmar
        debug -- if True, publish fake data, else, publish real data (Not yet implemented)
        '''
        line = self.ser.readline()
        if "," in str(line):
            try:
                msg = pynmea2.parse(line)
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
                rospy.loginfo("[airmar] Error!")
                rospy.loginfo(str(e))



    def airmar_pub(self, pub):
        '''
        A publisher that outputs airmar data at a rate of 10Hz
        '''
        airmar_data_msg = AirmarData()
        airmar_data_msg.heading = self.heading
        airmar_data_msg.amrRoll = self.amrRoll
        airmar_data_msg.apWndDir = self.apWndDir
        airmar_data_msg.apWndSpd = self.apWndSpd
        airmar_data_msg.cog = self.cog
        airmar_data_msg.sog = self.sog
        airmar_data_msg.truWndSpd = self.truWndSpd
        airmar_data_msg.truWndDir = self.truWndDir
        airmar_data_msg.lat = self.lat
        airmar_data_msg.long = self.long      #FIXME: does 'long' overwrite a python keyword?

        pub.publish(airmar_data_msg)

if __name__ == '__main__':
    try:
        #Init the ros node
        rospy.init_node('airmar');
        am = Airmar()
        rospy.loginfo("[airmar] Started airmar node!")
        
        tm = rospy.get_time()

        while not rospy.is_shutdown():
            am.update_data() # Get the latest data from the airmar
            if (rospy.get_time() - tm > am.pubInterval):
                am.airmar_pub(pub, lat_long_pub) # Publish the latest data
                tm = rospy.get_time()

    except Exception as e:
        print e

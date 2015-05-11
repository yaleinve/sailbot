#!/usr/bin/env python

import rospy
import random
import math
import serial
import pynmea2
import pdb
import time

from airmar.msg import AirmarData   #NOTE THAT THIS NO LONGER CONTAINS TRUE WIND FIELDS!!
from sensor_msgs.msg import NavSatFix

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
        self.ser = serial.Serial('/dev/ttyUSB1', 4800, timeout=1)
        self.ser.readline()


        # Default publishing rate is 10Hz
        self.pub_rate = rospy.Rate(rospy.get_param('rate', 2))

    def update_data(self, debug=False):
        '''
        Fetches the latest data off the serial wire from the Airmar
        debug -- if True, publish fake data, else, publish real data
        '''
        line = self.ser.readline()
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
                rospy.loginfo("[airmar] Error!")
                rospy.loginfo(str(e))



    def airmar_pub(self, pub, pub2):
        '''
        A publisher that outputs airmar data at a rate of 10Hz
        '''
        # NOTE: HEADING HAS NOT BEEN UPDATED YET IN update_data
        airmar_data_msg = AirmarData()
        airmar_data_msg.heading = self.heading
        airmar_data_msg.amrRoll = self.amrRoll
        airmar_data_msg.apWndDir = self.apWndDir
        airmar_data_msg.apWndSpd = self.apWndSpd
        airmar_data_msg.cog = self.cog
        airmar_data_msg.sog = self.sog
        airmar_data_msg.truWndSpd = self.truWndSpd
        airmar_data_msg.truWndDir = self.truWndDir

        # The following data is not currently in the AirmarData message spec
        # As such, it is not being published, but it is still being collected
        '''airmar_data_msg.cog = self.cog
        airmar_data_msg.lat = self.lat
        airmar_data_msg.long = self.long
        airmar_data_msg.sog = self.sog
        airmar_data_msg.truWndDir = self.truWndDir
        airmar_data_msg.truWndSpd = self.truWndSpd'''

        lat_long_msg = NavSatFix()
        lat_long_msg.latitude = self.lat
        lat_long_msg.longitude = self.long

        pub.publish(airmar_data_msg)
        pub2.publish(lat_long_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('airmar');
        am = Airmar()

        pub = rospy.Publisher('/airmar_data', AirmarData, queue_size = 10)
        lat_long_pub = rospy.Publisher('fix', NavSatFix, queue_size = 10)
        rospy.loginfo("[airmar] Started airmar node!")
        tm = rospy.get_time()

        while not rospy.is_shutdown():
            # TODO: Implement actual publisher instead of debugging one
            am.update_data() # Get the latest data from the airmar
            # add the debug = True flag to enter debug mode
            if (rospy.get_time() - tm > 0.5):
                am.airmar_pub(pub, lat_long_pub) # Publish the latest data
                tm = rospy.get_time()
            #am.pub_rate.sleep()
    except Exception as e:
        print e

#!/usr/bin/env python

import rospy
import random
import math
import serial
import pynmea2
import pdb

from airmar.msg import AirmarData   #NOTE THAT THIS NO LONGER CONTAINS TRUE WIND FIELDS!!

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
        self.ser = serial.Serial('/dev/ttyUSB0', 4800, timeout=1)
        self.ser.readline()


        # Default publishing rate is 10Hz
        self.pub_rate = rospy.Rate(rospy.get_param('rate', 10))

    def update_data(self, debug=False):
        '''
        Fetches the latest data off the serial wire from the Airmar
        debug -- if True, publish fake data, else, publish real data
        '''
        line = self.ser.readline()
        try:
            msg = pynmea2.parse(line)
            # self.lat = msg.latitude
            # self.long = msg.longitude
            if msg.sentence_type == 'MWV':
                self.apWndSpd = msg.wind_speed
                self.wndUnits = msg.wind_speed_units
                self.apWndDir = msg.wind_angle
            elif msg.sentence_type == 'HDT': # change to HDG for magnetic heading
                self.heading = msg.heading
            elif msg.sentence_type == 'VTG':
                self.cog = msg.true_track # change to mag_track for magnetic cog
                self.sog = msg.spd_over_grnd_kts # can also support kmph
            elif msg.sentence_type == 'GGA':
                self.lat = msg.latitude
                self.long = msg.longitude

        except Exception, e:
            # print e
            rospy.loginfo("[airmar] Error!")



    def airmar_pub(self, pub):
        '''
        A publisher that outputs airmar data at a rate of 10Hz
        '''
        # NOTE: HEADING HAS NOT BEEN UPDATED YET IN update_data
        airmar_data_msg = AirmarData()
        airmar_data_msg.heading = self.heading
        airmar_data_msg.amrRoll = self.amrRoll
        airmar_data_msg.apWndDir = self.apWndDir
        airmar_data_msg.apWndSpd = self.apWndSpd
        
        # The following data is not currently in the AirmarData message spec
        # As such, it is not being published, but it is still being collected
        '''airmar_data_msg.cog = self.cog
        airmar_data_msg.lat = self.lat
        airmar_data_msg.long = self.long
        airmar_data_msg.sog = self.sog
        airmar_data_msg.truWndDir = self.truWndDir
        airmar_data_msg.truWndSpd = self.truWndSpd'''

        pub.publish(airmar_data_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('airmar');
        am = Airmar()

        pub = rospy.Publisher('/airmar_data', AirmarData, queue_size = 10)
        rospy.loginfo("[airmar] Started airmar node!")

        while not rospy.is_shutdown():
            # TODO: Implement actual publisher instead of debugging one
            am.update_data() # Get the latest data from the airmar
            # add the debug = True flag to enter debug mode
            am.airmar_pub(pub) # Publish the latest data
            am.pub_rate.sleep()
    except Exception as e:
        print e
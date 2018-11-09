#!/usr/bin/env python

import rospy
import random
import math

from airmar.msg import AirmarData   #NOTE THAT THIS NO LONGER CONTAINS TRUE WIND FIELDS!!

class Airmar:
    def __init__(self):
        self.heading = 0
        self.amrRoll = 0
        #self.truWndDir = 0
        #self.truWndSpd = 0
        #self.cog = 0
        #self.lat = 0
        #self.long = 0
        #self.sog = 0

        # For debugging, sets a random phase offset so that apWndDir starts
        # in a different place from the heading.
        self.apWndDir = random.random()*360.0
        self.apWndSpd = 5 # Also for debugging

        # Default publishing rate is 10Hz
        self.pub_rate = rospy.Rate(rospy.get_param('rate', 10))

    def update_data(self, debug=False):
        '''
        Fetches the latest data off the serial wire from the Airmar

        debug -- if True, publish fake data, else, publish real data
        '''

        if debug:
            # adds += 0.5 degrees to the current heading
            self.heading = self.heading + (0.5 - random.random())
            self.heading = self.heading % 360 # degrees

            # adds += 0.25 degrees to the current roll angle
            self.amrRoll = self.amrRoll + 2.0*(0.5 - random.random())*0.25
            if self.amrRoll > 45:
                self.amrRoll = 45
            elif self.amrRoll < -45:
                self.amrRoll = -45
            '''
            # adds += 0.5 degrees to the current truWndDir
            self.truWndDir = self.truWndDir + (0.5 - random.random())
            self.truWndDir = self.truWndDir % 360 # degrees

            # adds += 0.5 units to the current truWndSpd TODO: Knots?
            self.truWndSpd = self.truWndSpd + (0.5 - random.random())
            if self.truWndSpd > 10:
                self.truWndSpd = 10
            elif self.truWndSpd < 0:
                self.truWndSpd = 0
            
            
            self.cog = self.cog
            self.lat = self.lat
            self.long = self.long
            self.sog = self.sog
            '''


    def airmar_pub(self, pub):
        '''
        A publisher that outputs airmar data at a rate of 10Hz
        '''

        airmar_data_msg = AirmarData()
        airmar_data_msg.heading = self.heading
        airmar_data_msg.amrRoll = self.amrRoll
        airmar_data_msg.apWndDir = self.apWndDir
        airmar_data_msg.apWndSpd = self.apWndSpd
        '''
        airmar_data_msg.cog = self.cog
        airmar_data_msg.lat = self.lat
        airmar_data_msg.long = self.long
        airmar_data_msg.sog = self.sog

        airmar_data_msg.truWndDir = self.truWndDir
        airmar_data_msg.truWndSpd = self.truWndSpd
        '''
        pub.publish(airmar_data_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('airmar');
        am = Airmar()

        pub = rospy.Publisher('airmar_data', AirmarData, queue_size = 10)
        rospy.loginfo("[airmar] Started airmar node!")

        while not rospy.is_shutdown():
            # TODO: Implement actual publisher instead of debugging one
            am.update_data(debug=True) # Get the latest data from the airmar
            am.airmar_pub(pub) # Publish the latest data
            am.pub_rate.sleep()
    except Exception as e:
        print e

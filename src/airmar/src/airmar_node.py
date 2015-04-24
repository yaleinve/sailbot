#!/usr/bin/env python

import rospy
import random
import math

from airmar.msg import AirmarData

class Airmar:
    def __init__(self):
        self.heading = 0
        self.amrRoll = 0
        self.apWndDir = 0
        self.apWndSpd = 0
        self.cog = 0
        self.lat = 0
        self.long = 0
        self.sog = 0
        self.truWndDir = 0
        self.truWndSpd = 0

        # Default publishing rate is 10Hz
        self.pub_rate = rospy.Rate(rospy.get_param('rate', 10))

    def update_data(self, debug=False):
        '''
        Fetches the latest data off the serial wire from the Airmar

        debug -- if True, publish fake data, else, publish real data
        '''

        if debug:
            # adds += 5 degrees to the current heading
            self.heading = self.heading + 2.0*(0.5 - random.random())*0.5
            self.heading = self.heading % 360 # degrees
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
        airmar_data_msg.lat = self.lat
        airmar_data_msg.long = self.long
        airmar_data_msg.sog = self.sog
        airmar_data_msg.truWndDir = self.truWndDir
        airmar_data_msg.truWndSpd = self.truWndSpd

        pub.publish(airmar_data_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('airmar');
        am = Airmar()

        pub = rospy.Publisher('airmar', AirmarData, queue_size = 10)

        while not rospy.is_shutdown():
            # TODO: Implement actual publisher instead of debugging one
            am.update_data(debug=True) # Get the latest data from the airmar
            am.airmar_pub(pub) # Publish the latest data
            am.pub_rate.sleep()
    except:
        print "what is in a name"

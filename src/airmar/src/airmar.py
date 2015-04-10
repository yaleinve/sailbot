#!/usr/bin/env python

import rospy

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

    def fake_pub(self, pub):
        '''
        A debugging publisher that outputs fake airmar data at a rate of 10Hz
        '''




if __name__ == '__main__':
    try:
        rospy.init_node('airmar');
        am = Airmar()

        pub = rospy.Publisher('airmar', AirmarData, queue_size = 10)

        while not rospy.is_shutdown():
            am.fake_pub(pub);
            am.pub_rate.sleep()

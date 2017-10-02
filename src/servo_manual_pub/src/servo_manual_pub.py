#!/usr/bin/env python

#Updated by Eric Anderson 12/15 to adapt for Edison and recent arch changes,
#namely that we rely solely on airmar (and camera) for sensing - no navsat

import rospy
import pdb
import time
#import mraa

from sails_rudder.msg import SailsRudderPos

if __name__ == '__main__':
    try:
        #Init the ros node
        rospy.init_node('serv_manual_pub')
        pub = rospy.Publisher('/sails_rudder_pos', SailsRudderPos, queue_size = 10)
        while not rospy.is_shutdown():
            manmsg = SailsRudderPos()
            val = input("Enter a main angle:")
            print("Main value received was : " + str(val) + '\n')
            manmsg.mainPos = val
            val = input("Enter a jib angle:")
            print("Jib value received was : " + str(val) + '\n')
            manmsg.jibPos = val
            val = input("Enter a rudder angle:")
            print("Rudder value received was : " + str(val) + '\n')
            manmsg.rudderPos = val
            print("Publishing now\n...\n")
            pub.publish(manmsg)

    except Exception as e:
        print e

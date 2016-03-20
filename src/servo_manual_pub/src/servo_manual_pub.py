#!/usr/bin/env python

#Updated by Eric Anderson 12/15 to adapt for Edison and recent arch changes,
#namely that we rely solely on airmar (and camera) for sensing - no navsat

import rospy
import pdb
import time
#import mraa

from servo_control.msg import ServoPos

if __name__ == '__main__':
    try:
        #Init the ros node
        rospy.init_node('serv_manual_pub')
        pub = rospy.Publisher('/servo_pos', ServoPos, queue_size = 10)
        while not rospy.is_shutdown():
            manmsg = ServoPos()
            val = input("Enter a main angle:")
            print("Main value received was : " + str(val) + '\n')
            manmsg.main_angle = val
            val = input("Enter a jib angle:")
            print("Jib value received was : " + str(val) + '\n')
            manmsg.jib_angle = val
            val = input("Enter a rudder angle:")
            print("Rudder value received was : " + str(val) + '\n')
            manmsg.rudder_angle = val
            print("Publishing now\n...\n")
            pub.publish(manmsg)

    except Exception as e:
        print e

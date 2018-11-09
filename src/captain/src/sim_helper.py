#!/usr/bin/python
# captain/sim_helper.py  Linc Berkley Dec 17
# Provides services for captain_runner to access simulated hardware

import rospy

from captain.srv import *


def handle_set_relays_auto(auto):
    return SetRelaysAutoResponse()


def handle_get_auto_pin(req):
    return True


if __name__ == '__main__':
    rospy.init_node("captain_hardware_helper")

    s1 = rospy.Service('set_relays_auto', SetRelaysAuto, handle_set_relays_auto)
    s2 = rospy.Service('get_auto_pin', GetAutoPin, handle_get_auto_pin)

    rospy.spin()

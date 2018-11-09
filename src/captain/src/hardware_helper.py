#!/usr/bin/python
# captain/hardware_helper.py  Linc Berkley Dec 17
# Provides services for captain_runner to access real hardware

import rospy
import mraa

from captain.srv import *


def handle_set_relays_auto(auto):
    sailRelayPin.write(int(auto))
    rudderRelayPin.write(int(auto))
    return SetRelaysAutoResponse()


def handle_get_auto_pin(req):
    return not (backAutoDivide <= autonomousPin.read() <= auxDivide)


if __name__ == "__main__":
    try:
        rospy.init_node("captain_hardware_helper")

        # Autonomous pins from RC
        autonomousPin = mraa.Aio(1)  # The analog pin number for aux 1
        auxDivide = 60  # A good dividing line (in 1024 bit adc units) to determine between high and low switch)
        backAutoDivide = 20

        # Relay pins
        sailRelayPin = mraa.Gpio(4)  # Pin 4 is sail relays
        sailRelayPin.dir(mraa.DIR_OUT)
        rudderRelayPin = mraa.Gpio(2)  # Pin 2 is rudder relay
        rudderRelayPin.dir(mraa.DIR_OUT)

        s1 = rospy.Service('set_relays_auto', SetRelaysAuto, handle_set_relays_auto)
        s2 = rospy.Service('get_auto_pin', GetAutoPin, handle_get_auto_pin)
        rospy.loginfo("[captin_hardware_helper] Started")
        rospy.spin()

    except KeyboardInterrupt:
        sailRelayPin.write(0)
        rudderRelayPin.write(0)

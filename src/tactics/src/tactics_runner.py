#!/usr/bin/env python2.7

import rospy
import time
from compassCalc import *
from gpsCalc import *

#Import the signatures/headers for the message types we have created
from tactics.msg import NavTargets
from airmar.msg import AirmarData
from speed_calculator.msg import SpeedStats
from captain.msg import LegInfo


tactics_pub = rospy.Publisher("/nav_targets", NavTargets, queue_size=10)
leg = None
speed_stats = None
airmar = None


tack = 'port'
lastTackTime = time.time()


# What is the minimum angle from the wind (when facing into it) we must maintain?
UPWIND_THRESHOLD = 40.0  # degrees

# How far downwind do we want to run?
DOWNWIND_THRESHOLD = 165.0  # degrees (away from the wind)

# How long do we have to wait to tack, at minimum?
TACK_DELAY = 10.0  # seconds


def initGlobals():
    global lastTack

    lastTack = 0.0



def publish_tactics():
    rospy.loginfo("[tactics] publishTactics()")
    global lastTack, lastTargetHeading, pointOfSail, tack

    if leg is None:
        rospy.loginfo("[tactics] leg is null")
        return
    if airmar is None:
        rospy.loginfo("[tactics] airmar is null")
        return
    if speed_stats is None:
        rospy.loginfo("[tactics] speed_stats is null")
        return

    target_course = gpsBearing(airmar.lat, airmar.long, leg.end_lat, leg.end_long)
    target_range = gpsDistance(airmar.lat, airmar.long, leg.end_lat, leg.end_long)
    wind_targ_angle = compass_diff(target_course, airmar.truWndDir)

    # When do we want to tack?
    # Ideally we wouldn't tack at all because we lose momentum. But when the target's
    # very upwind we don't have a choice; we can't sail directly upwind.
    # Instead, we tack when we leave the cross track area, making a zigzag pattern.

    # Important context: XTE, or cross track error, is the distance (in meters) from
    # the line between the initial position and the target.
    # It's negative when you're to the left of that line, when it's oriented so that the target is at the top.

    if abs(wind_targ_angle) <= UPWIND_THRESHOLD:
        # we're sailing into the wind.
        point_of_sail = "Pointing (%s)" % tack

        if speed_stats.xte < leg.xte_min:  # we're out of XTE on the left
            tack = 'port'  # put the wind on the left of us, sailing right
            rospy.loginfo("[tactics] Tack to port")

        if speed_stats.xte > leg.xte_max:  # we're out of XTE on the right
            tack = 'starboard'  # put the wind on the right of us, sailing left
            rospy.loginfo("[tactics] Tack to starboard")

        # For now we sail into the wind until the target isn't in the wind anymore.
        result_heading = airmar.truWndDir + (UPWIND_THRESHOLD if tack == 'port' else -UPWIND_THRESHOLD)
    elif abs(wind_targ_angle) <= UPWIND_THRESHOLD:
        # we're sailing roughly perpendicular to the wind.
        # We can just head directly to the target and sails_rudder is smart enough to handle the sail.

        point_of_sail = "Hauling / Beating"
        result_heading = target_course
    else:
        # we don't want to sail directly downwind either
        point_of_sail = "Running (%s)" % tack

        if speed_stats.xte < leg.xte_min:  # we're out of XTE on the left
            tack = 'port'  # put the wind on the left of us, sailing right
            rospy.loginfo("[tactics] Jibe to port")

        if speed_stats.xte > leg.xte_max:  # we're out of XTE on the right
            tack = 'starboard'  # put the wind on the right of us, sailing left
            rospy.loginfo("[tactics] Jibe to starboard")

        # the result will be sailing downwind until we get close enough that the target isn't directly in the
        # wind, then we turn around into the target
        result_heading = airmar.truWndDir - (DOWNWIND_THRESHOLD if tack == 'port' else -DOWNWIND_THRESHOLD)

    msg = NavTargets()

    msg.pointOfSail = pointOfSail
    msg.targetHeading = result_heading
    msg.targetCourse = target_course
    msg.targetRange = target_range

    tactics_pub.publish(msg)


def airmar_callback(data):
    global airmar, target_course, target_range
    airmar = data

    if leg is None:
        return


def speed_stats_callback(data):

    global speed_stats
    speed_stats = data

    publish_tactics()  # We publish every time the airmar updates


def leg_info_callback(data):
    rospy.loginfo("[tactics] leg_info_callback()")
    global leg
    leg = data


def listen():
    initGlobals()

    rospy.init_node("tactics")
    rospy.Subscriber("/airmar_data", AirmarData, airmar_callback)
    rospy.Subscriber("/speed_stats", SpeedStats, speed_stats_callback)
    rospy.Subscriber("/leg_info", LegInfo, leg_info_callback)
    rospy.loginfo("[tactics] All subscribed, tactics has started!")

    rospy.spin()


if __name__ == "__main__":
    listen()

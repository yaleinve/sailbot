#!/usr/bin/env python
# tactics.py              Eric Anderson Mar 2015

#Import statements
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

# What is the minimum angle from the wind (when facing into it) we must maintain?
UPWIND_THRESHOLD = 45.0  # degrees

# How far downwind can we point without sailing too unsafely with the wind?
DOWNWIND_THRESHOLD = 15.0  # degrees

# How long do we have to wait to tack, at minimum?
TACK_DELAY = 10.0  # seconds


def initGlobals():
    global target_course
    global target_range
    global pointOfSail
    global lastTack
    global lastTargetHeading

    target_course = 0.0
    target_range = 0.0
    pointOfSail = ""
    lastTack = 0.0
    lastTargetHeading = 0.0



def publish_tactics():
    global lastTack  # The only global we'll write to
    global lastTargetHeading

    if leg is None or airmar is None or speed_stats is None:
        return

    # Constants for Racht. MUST BE EMPIRICALLY DETERMINED
    pointing_angle = 50.0   # Can't point closer than 50 degrees to wind
    running_angle = 165.0   # Don't want to sail deeper than this
    delayBetweenTacks = 10.0    # Don't tack if tacked within the last x seconds
    # talk to eric what would be reasonable for this delay.
    # Be careful that this doesn't accidentally sail you out of box for station keeping!!

    #ACTUAL ALGORITHM:

    diff = compass_diff(target_course, airmar.truWndDir)  #From where we want to go to the wind

    #Reaching Mode is default
    targetHeading = target_course
    pointOfSail = "Reaching"
    onStbd = (compass_diff(airmar.heading,airmar.truWndDir ) > 0.0)

    stbd = 0.0
    port = 0.0

    #Beating Mode
    if abs(diff) < pointing_angle:
        pointOfSail = "Beating"
        stbd = (airmar.truWndDir - pointing_angle)  % 360   #Define airmar.headings of both tacks
        port = (airmar.truWndDir + pointing_angle)  % 360
        if abs(compass_diff(airmar.heading, stbd)) >= abs(compass_diff(airmar.heading, port)):  #Which one are we closer to?
            targetHeading = port
        else:
            targetHeading = stbd

    #Running mode
    elif abs(diff) > running_angle:
        pointOfSail = "Running"
        stbd = (airmar.truWndDir - running_angle) % 360   #Define airmar.headings of both tacks
        port = (airmar.truWndDir + running_angle) % 360
        if abs(compass_diff(airmar.heading, stbd)) >= abs(compass_diff(airmar.heading, port)):  #Which one are we closer to?
            targetHeading = port
        else:
            targetHeading = stbd

    # I think this algorithm might have lots of weird edge cases:
    # What if on a reach but slide below course to the point you have to beat?
    # What if you sail past your destination on a beat and start running?
    # Implement Tacking
    if (time.time()-lastTack > delayBetweenTacks):  #Supress frequent tacking
        if pointOfSail == "Running":                  #Transitions are reveresed for
            if onStbd and speed_stats.xte > leg.xte_max:                 #Beating and Running
                rospy.loginfo("[tactics] Jibing to starboard");
                targetHeading = port                      #Do we want to signal a jibe????
                lastTack = time.time()
            elif (not onStbd) and speed_stats.xte < leg.xte_min:
                rospy.loginfo("[tactics] Jibing to port");
                targetHeading = stbd
                lastTack = time.time()
            lastTargetHeading = targetHeading
        elif pointOfSail == "Beating":
            if onStbd and speed_stats.xte < leg.xte_min:
                rospy.loginfo("[tactics] Tacking to starboard");
                targetHeading = port
                lastTack = time.time()
            elif (not onStbd) and speed_stats.xte > leg.xte_max:
                rospy.loginfo("[tactics] Tacking to port");
                targetHeading = stbd
                lastTack = time.time()
            lastTargetHeading = targetHeading
    else:
        targetHeading = lastTargetHeading

    msg = NavTargets()
    msg.pointOfSail = pointOfSail
    msg.targetHeading = targetHeading
    msg.targetCourse = target_course
    msg.targetRange = target_range

    tactics_pub.publish(msg)

def airmar_callback(data):
    global airmar, target_course, target_range
    airmar = data

    if leg is None:
        return

    # Where is the target, relative to us? (by angle and by distance)
    target_course = gpsBearing(airmar.lat, airmar.long, leg.end_lat, leg.end_long)
    target_range = gpsDistance(airmar.lat, airmar.long, leg.end_lat, leg.end_long)

def speed_stats_callback(data):
    global speed_stats
    speed_stats = data

    publish_tactics()  # We publish every time the airmar updates


def leg_info_callback(data):
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

#!/usr/bin/env python
# tactics.py              Eric Anderson Mar 2015

#Import statements
import roslib
import rospy
import sys
import time
from compassCalc import *
from gpsCalc import *

#Import the signatures/headers for the message types we have created
from tactics.msg import NavTargets
from airmar.msg import AirmarData
from captain.msg import LegInfo

pub_tactics = rospy.Publisher("/nav_targets", NavTargets, queue_size = 10)

def initGlobals():
  global target_course
  global target_range
  global heading
  global apWndDir
  global apWndSpd
  global xte
  global cog
  global sog
  global xteMax
  global xteMin
  global vmg
  global vmgUp
  global pointOfSail
  global lastTack
  global truWndDir
  global legEndLat
  global legEndLong
  global currentLat
  global currentLong


  target_course = 0.0
  target_range = 0.0
  heading = 0.0
  apWndDir = 0.0
  apWndSpd = 0.0
  xte = 0.0
  cog = 0.0
  sog = 0.0
  xteMax = 0.0
  xteMin = 0.0
  vmg = 0.0
  vmgUp = 0.0
  pointOfSail = ""
  lastTack = time.time()
  truWndDir = 0.0
  currentLat = 0.0
  currentLong= 0.0
  legEndLat = 0.0
  legEndLong = 0.0



#Publish tactics output message, target_heading.  This function contains the actual algorithm.
def publish_tactics():
  global lastTack  #The only global we'll write to

  #Constants for Racht. MUST BE EMPIRICALLY DETERMINED
  pointing_angle = 50.0   #Can't point closer than 50 degrees to wind
  running_angle = 165.0   #Don't want to sail deeper than this
  delayBetweenTacks = 10.0 #Don't tack if tacked within the last x seconds
                           #talk to eric what would be reasonable for this delay.
                           #Be careful that this doesn't accidentally sail you out of box for station keeping!!

  #ACTUAL ALGORITHM:

  diff = compass_diff(target_course,truWndDir)  #From where we want to go to the wind
  #rospy.loginfo("diff is : " + str(diff))                                                            # (remember wind vector orientation)

  #Reaching Mode is default
  targetHeading = target_course
  pointOfSail = "Reaching"
  onStbd = (compass_diff(heading,truWndDir ) > 0.0)

  stbd = 0.0
  port = 0.0

  #Beating Mode
  if abs(diff) < pointing_angle:
    pointOfSail = "Beating"
    stbd = (truWndDir - pointing_angle)  % 360   #Define headings of both tacks
    port = (truWndDir + pointing_angle)  % 360
    if abs(compass_diff(heading, stbd)) >= abs(compass_diff(heading, port)):  #Which one are we closer to?
      targetHeading = port
    else:
      targetHeading = stbd

  #Running mode
  elif abs(diff) > running_angle:
    pointOfSail = "Running"
    stbd = (truWndDir - running_angle) % 360   #Define headings of both tacks
    port = (truWndDir + running_angle) % 360
    if abs(compass_diff(heading, stbd)) >= abs(compass_diff(heading, port)):  #Which one are we closer to?
      targetHeading = port
    else:
      targetHeading = stbd


  #I think this algorithm might have lots of weird edge cases:
  #What if on a reach but slide below course to the point you have to beat?
  #What if you sail past your destination on a beat and start running?

  #Implement Tacking
  if (time.time()-lastTack > delayBetweenTacks):  #Supress frequent tacking
    if pointOfSail == "Running":                  #Transitions are reveresed for
      if onStbd and xte > xteMax:                 #Beating and Running
        targetHeading = port                      #Do we want to signal a jibe????
        lastTack = time.time()
      elif (not onStbd) and xte < xteMin:
        targetHeading = stbd
        lastTack = time.time()
    elif pointOfSail == "Beating":
      if onStbd and xte < xteMin:
        targetHeading = port
        lastTack = time.time()
      elif (not onStbd) and xte > xteMax:
        targetHeading = stbd
        lastTack = time.time()

  msg = NavTargets()  #Instantiate a message
  msg.pointOfSail = pointOfSail  #From globals
  msg.targetHeading = targetHeading
  msg.targetCourse = target_course
  msg.targetRange = target_range
  pub_tactics.publish(msg) #Publish the message


#Put the data from the airmar message into global variables
def airmar_callback(data):
  global heading
  global apWndSpd
  global apWndDir
  global xte
  global vmg
  global vmgUp
  global cog
  global sog
  global truWndDir
  global currentLat
  global currentLong
  global target_course
  global target_range

  heading  = data.heading
  apWndSpd = data.apWndSpd
  apWndDir = data.apWndDir
  xte = data.XTE
  vmg = data.VMG
  vmgUp = data.VMGup
  cog = data.cog
  sog = data.sog
  truWndDir = data.truWndDir
  currentLat = data.lat
  currentLong = data.long

  #This is all that the old navigator node did:
  target_course = gpsBearing(currentLat, currentLong, legEndLat, legEndLong)
  target_range =  gpsDistance(currentLat, currentLong, legEndLat, legEndLong)

  publish_tactics()  #We publish every time the airmar updates

#Only need a few things from leg_info
def leg_info_callback(data):
  global xteMax
  global xteMin
  global legEndLat
  global legEndLong

  xteMin = data.xte_min
  xteMax = data.xte_max
  legEndLat = data.end_lat
  legEndLong = data.end_long

def listener():
  initGlobals()

  rospy.init_node("tactics")  #Must init node to subscribe
  rospy.Subscriber("/airmar_data", AirmarData, airmar_callback)
  rospy.Subscriber("/leg_info", LegInfo, leg_info_callback)
  rospy.loginfo("[tactics] All subscribed, tactics has started!")

  rospy.spin()


if __name__ == "__main__":
  listener() 	#Listen to our subscriptions

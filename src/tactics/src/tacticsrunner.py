#!/usr/bin/env python
# tactics.py 							Eric Anderson Mar 2015

#Import statements
import roslib
import rospy
import sys
import time

#Import the signatures/headers for the message types we have created
from tactics.msg import TargetHeading 
from navigator.msg import TargetCourse
from airmar.msg import AirmarData 
from speed_calculator.msg import SpeedStats
from captain.msg import LegInfo

#global variables  TODO must initialize to some value!!
target_course = 0.0  #From navigator

heading = 0.0     #From airmar_data
cog = 0.0
sog = 0.0
apWndDir = 0.0
apWndSpd = 0.0

xte = 0.0         #From leg_info
xteMax = 0.0
xteMin = 0.0

#Below not currently used but could be in later iterations
vmg = 0.0
vmgUp = 0.0
target_range = 0.0


#Internal State Variables
pointOfSail = ""       #'Enum' consisting of Beating, Reaching, Running
targetHeading = 0.0          #Our goal heading (to be published)
onStbd = False         #The tack we're on
lastTack = time.time() #Time of last tack

#Returns the shortest signed difference between two compass headings.
#Examples: compass_diff(359.0,2.0) = 3.0, compass_diff(2.0,359.0) = -3.0
#Breaks ties by turning to the right 
#tested and working 4/17/15
def compass_diff(head1,head2):
  d = head2-head1   #raw difference
  if d >= 0.0:  #head2 is on same compass 'rotation' as head1 and ahead of head1 in a CW sense
    if d <= 180.0:  #head1 is just a little behind head2
      return d
    else:
      return d - 360 #head1 is very far behind head2, so easier to go CCW (must return negative!)
  else:  #d < 0.0, head1 is ahead of head 2 on same 'rotation'
    if d >= -180.0:  #head1 is only a little ahead of head2
      return d
    else:
      return d + 360 #shorter to go CW to get there, so must be positive


#Publish tactics output message, target_heading.  This function contains the actual algorithm.
def publish_tactics():
  #Constants for Racht. MUST BE EMPIRICALLY DETERMINED
  pointing_angle = 50.0   #Can't point closer than 50 degrees to wind
  running_angle = 165.0   #Don't want to sail deeper than this
  delayBetweenTacks = 3.0 #Don't tack if tacked within the last x seconds
 

  #ACTUAL ALGORITHM:

  diff = compass_diff(target_course,apWndDir)  #From where we want to go to the wind

  #Reaching Mode is default
  global targetHeading
  global pointOfSail
  global onStbd
  targetHeading = target_course
  pointOfSail = "Reaching"  
  onStbd = (compass_diff(heading,apWndDir) > 0.0)

  #Beating Mode
  if abs(diff) < pointing_angle:
    pointOfSail = "Beating"
    stbd = (apWndDir - pointing_angle) % 360.0  #Define headings of both tacks 
    port = (apWndDir + pointing_angle) % 360.0
    
    if abs(compass_diff(heading, stbd)) <= pointing_angle:  #Which one are we closer to?
      targetHeading = stbd
    else:
      targetHeading = port

  #Running mode
  elif abs(diff) > running_angle:  
    pointOfSail = "Running"
    stbd = (apWndDir - running_angle) % 360.0  #Define headings of both tacks 
    port = (apWndDir + running_angle) % 360.0
    
    if abs(compass_diff(heading,stbd)) < 180-running_angle:  #Which one are we closer to
      targetHeading = stbd
    else:
      targetHeading = port

  #Implement Tacking
  if (time.time()-lastTack > delayBetweenTacks):  #Supress frequent tacking
    if pointOfSail == "Running":                  #Transitions are reveresed for
      if onStbd and xte > xteMax:                 #Beating and Running
        targetHeading = port
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


  #Publish the results
  rospy.init_node("target_heading")  #Must reinit the node to publish
  
  #1st arg is topic name, 2nd is filename in /msg,  3rd is  ? 
  pub_stats = rospy.Publisher("/target_heading", 
      TargetHeading, queue_size = 10) 
  
  target_heading = TargetHeading()  #Instantiate a message
  target_heading.pointOfSail = pointOfSail  #From globals
  target_heading.targetHeading = targetHeading
  pub_stats.publish(target_heading) #Publish the message


#Put the data from the airmar message into global variables
def airmar_callback(data):
  global heading
  global cog
  global sog
  global apWndSpd
  global apWndDir

  heading  = data.heading
  cog = data.cog
  sog = data.sog
  apWndSpd = data.apWndSpd
  apWndDir = data.apWndDir
  publish_tactics()  #We publish every time the airmar updates

#Put the data from the target course message into global variables
def target_course_callback(data):
  global target_course
  global target_range
  target_course = data.course
  target_range = data.range

def speed_stats_callback(data):
  global xte
  global vmg
  global vmgUp
  xte = data.XTE
  vmg = data.VMG
  vmgUp = data.VMGup

#Only need a few things from leg_info
def leg_info_callback(data):
  global xteMax
  global xteMin
  xteMin = data.xte_min
  xteMax = data.xte_max

def listener():
  rospy.init_node("tactics")  #Must init node to subscribe

  rospy.Subscriber("navigator", TargetCourse, target_course_callback)
  rospy.Subscriber("airmar_data", AirmarData, airmar_callback)
  rospy.Subscriber("speed_stats", SpeedStats, speed_stats_callback)
  rospy.Subscriber("leg_info", LegInfo, leg_info_callback)

  rospy.spin()


if __name__ == "__main__":
  listener() 	#Listen to our subscriptions
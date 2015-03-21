#!/usr/bin/env python
#captain.py                                     Eric Anderson Mar 15
#Implements the captain node as described in the  specs
#TODO check that all queue additions are 4-ples with xtemin/max and clean up airmar callback function


import time
import roslib
import rospy
import sys
import queue
#Need a math import statement?

from gpsCalc import *             #import all the gps functions
from captain.msg import LegInfo
from airmar.msg import AirmarData #TODO not yet implemented.
from competition_info.msg import CompetitionInfo #TODO not yet implemented.
from std_msgs.msg import Bool #the bool for manual mode

#====== Default values on initialization ======#
currentLat = 0.0      #From the airmar
currentLong = 0.0
apWndSpd = 0.0
apWndDir = 0.0
truWndSpd = 0.0
truWndDir = 0.0

compMode = ""         #From competition_info
gpsLat1 = 0.0
gpsLong1 = 0.0
gpsLat2 = 0.0
gpsLong2 = 0.0
gpsLat3 = 0.0
gpsLong3 = 0.0
gpsLat4 = 0.0
gpsLong4 = 0.0
angle = 0.0

isManual = False     #From manual ms

#Internal State
legQueue = queue.Queue(maxsize=0)  #For leg sequencing
timeSinceLastPublish = 0.0
beginLat = 0.0
beginLong = 0.0
endLat = 0.0
endLong = 0.0
xteMin = 0.0
xteMax = 0.0
newSeq = True  #A flag to know whether we're starting a new sequence of legs
legArrivalTol = 2.0 #How close do we have to  get to waypoint to have "arrived"
                    #~~Must be empirically determined, set to 2.0m for now

def publish_captain():
  rospy.init_node("captain")
  pub_leg = rospy.Publisher("/leg_info", LegInfo, queue_size = 10)    
  leg_info = LegInfo()
  leg_info.begin_lat = beginLat
  leg_info.begin_long = beginLong
  leg_info.leg_course = gpsBearing(beginLat, beginLong, endLat, endLong)
  leg_info.end_lat = endLat
  leg_info.end_long = endLong
  leg_info.xte_min = xteMin
  leg_info.xte_max = xteMax
  pub_leg.publish(leg_info)

#Updates the state variables if we have copmpleted a leg or rerouted
def checkLeg():
  d = gpsDistance(currentLat,currentLong,endLat,endLong)
  if newSeq or d < legArrivalTol:      #If new or completed leg, we must update
    global newSeq = False
    global beginLat = currentLat
    global beginLong = currentLong
    
    if legQueue.empty():
      #Error, don't crash!!!!
    end = legQueue.get()
    
    if len(end) != 3:
      #Error, don't crash!

    global endLat = end[0]
    global endLong = end[1]
    global xteMin = end[2] if end[2] < 0.0 else -100.0  #Error checking
    global xteMax = end[3] if end[3] > 0.0 else 100.0
    publish_captain()


#Loads the leg Queue with appropriate legs given competition_info messages
def loadLegQueue():
  global legQueue = queue.Queue(maxsize=0)  #Empty the queue
  global newSeq = True                      #We are starting over
  a = compMode
  if a ==   "Wait":                 #Stay in the same place so we can get there 
  elif a == "SailToPoint":          #Sail to a gps target
    legQueue.put((gpsLat1,gpsLong1))#Insert a tuple gps loc
  elif a == "MaintainHeading":      #Sail a constant compass direction forever
    loc = gpsVectorOffset(currentLat,currentLong,angle,100000)
    leqQueue.put(loc)               #Sail 100km in direction of angle
  elif a == "MaintainPointOfSail":  #Sail a constant angle to the wind forever
    course = (truWndDir + angle) %360.0   #The course relative to the wind
    loc = gpsVectorOffset(currentLat,currentLong,course,100000)
    leqQueue.put(loc)               #Sail 100km in direction of angle
  elif a == "RoundAndReturn":       #Round a mark at gps1 and return to gps2
    #Create four legs forming a diamond around gps 1
  elif a == "StationKeeping":       #Stay within box created by 4 gps points
    #Complicated alg
  else:
    global compMode = "Wait"        #If invalid input, do nothing
    loadLegQueue()
  checkLeg()
  publish_captain()                 #If there's a new route, we should publish it!


def airmar_callback(data):
  global currentLat = data.lat
  global currentLong = data.long
  global apWndSpd = data.apWndSpd
  global apWndDir = data.apWndDir
  global truWndSpd = data.truWndSpd
  global truWndDir = data.truWndDir
  
  checkLeg()

  #avoid publishing every time we get data from the airmar. Every 10 seconds maybe
  if((time.time() - timeSinceLastPublish) >= 10.0):
    publish_captain()
    global timeSinceLastPublish = time.time()


#Every time new competition info comes in, we must re-route everything 
#TODO competition message not yet implemented, so I will implicitly spec it
#here to suit my needs.  As it is human input, it has no dependencies  
def competition_info_callback(data):
  global legQueue = queue.Queue(maxsize=0)   #Create new leg queue
  #Switch on the competition mode but PYTHON HAS NO SWITCH!
  global compMode = data.compMode
  global gpsLat1 = data.gpsLat1
  global gpsLong1 = data.gpsLong1
  global gpsLat2 = data.gpsLat2
  global gpsLong2 = data.gpsLong2
  global gpsLat3 = data.gpsLat3
  global gpsLong3 = data.gpsLong3
  global gpsLat4 = data.gpsLat4
  global gpsLong4 = data.gpsLong4
  global angle = (data.angle)%360.0   #Angle must be a valid  compass bearing

  loadLegQueue()      #Recalculate the leg queue every time we get a new
                      #message from competition_info


#going to be sent from the controller probably (some switch that changes between autonomous and manual)
def manual_callback(data):
  global isManual = data.data #the boolean that describes if we are in autonomous or in manual mode.


def listener():
  rospy.init_node("captain")
  rospy.Subscriber("airmar_data", AirmarData, airmar_callback)
  rospy.Subscriber("competition_info", CompetitionInfo, competition_info_callback)
  rospy.Subscriber("manual_mode", Bool, manual_callback)

  rospy.spin() 

if __name__ == "__main__":
  listener()





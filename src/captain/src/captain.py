#!/usr/bin/env python
#captain.py                                     Eric Anderson Mar 15
#Implements the captain node as described in the  specs

#TO DO: Implement WAIT and StationKeeping

import time
import roslib
import rospy
import sys
import Queue

from gpsCalc import gpsCalc  #import all the gps functions
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
xteMin = 0.0
xteMax = 0.0

#Internal State
class Waypoint():       
  def __init__(self, wlat,wlong,xteMin,xteMax):
    self.wlat = wlat if abs(wlat) <=90.0 else 0.0
    self.wlong = wlong if abs(wlong) <= 180.0 else 0.0
    self.xteMin = xteMini if xteMin <= -1.0 else -100.0
    self.xteMax = xteMax if xteMax >= 1.0 else 100.0

legQueue = queue.Queue(maxsize=0)  #A queue of waypoints
beginLat = 0.0
beginLong = 0.0
end = None          #The end waypoint for this leg

legArrivalTol = 2.0 #How close do we have to  get to waypoint to have "arrived"
cautious = False  #~~Must be empirically determined, set to 2.0m for now
cautiousDistance = 10.0   #How far away from waypoint to we start being cautious
timeSinceLastCheck = 0.0

def publish_captain():
  rospy.init_node("captain")
  pub_leg = rospy.Publisher("/leg_info", LegInfo, queue_size = 10)    
  leg_info = LegInfo()
  #Invariant: end != None (b/c nec called from checkLeg())
  leg_info.begin_lat = beginLat
  leg_info.begin_long = beginLong
  leg_info.leg_course = gpsBearing(beginLat, beginLong, end.wlat, end.wLong)
  leg_info.end_lat = end.wlat
  leg_info.end_long = end.wlong
  leg_info.xte_min = end.xteMin
  leg_info.xte_max = end.xteMax
  pub_leg.publish(leg_info)

#Updates the state variables if we have completed a leg or rerouted
def checkLeg():
  global beginLat
  global beingLong
  global end
  global compMode
  global legQueue
  if end == None:         #If no current waypoint
    if legQueue.empty():    
      compMode = "Wait"
      loadLegQueue()      #Recursive call may result in two publications in
      return              #rapid succession, but this isn't bad I don't think
    else:
      beginLat = currentLat      #Next leg starts here
      beginLong = currentLong
      end = legQueue.get()
      publish_captain()   

  #Fall thru b/c what if next leg is at the same place as this leg?
  d = gpsDistance(currentLat,currentLong,end.wlat,end.wlong) #Recognize complete leg
  cautious = (d < cautiousDistance)     #Should we start polling more quickly?
  if d < legArrivalTol:
    end = None                 #Recursively get next leg
    checkLeg()

#Loads the leg Queue with appropriate legs given competition_info messages
def loadLegQueue():
  global legQueue
  global end
  global compMode
  legQueue = queue.Queue(maxsize=0)  #Empty the queue
  end = None                         #We are starting over
  a = compMode
  if a ==   "Wait":                 #Stay in the same place so we can get there 
    pass
  elif a == "SailToPoint":          #Sail to a gps target
    legQueue.put(Waypoint(gpsLat1,gpsLong1,xteMin,xteMax)) #Insert a gps loc
  elif a == "MaintainHeading":      #Sail a constant compass direction forever
    loc = gpsVectorOffset(currentLat,currentLong,angle,100000)
    leqQueue.put(Waypoint(loc[0],loc[1],xteMin,xteMax)) #Sail 100km in direction of angle
  elif a == "MaintainPointOfSail":        #Sail a constant angle to the wind forever
    course = (truWndDir + angle) %360.0   #The course relative to the wind
    loc = gpsVectorOffset(currentLat,currentLong,course,100000)
    leqQueue.put(Waypoint(loc[0],loc[1],xteMin,xteMax)) #Sail 100km in direction of angle
  elif a == "RoundAndReturn":       #Round a mark at gps1 and return to gps2
    brng = gpsBearing(currentLat,currentLong,gpsLat1,gpsLong2)
    loc1 = gpdVectorOffset(gpsLat1,gpsLong1, (brng+90)%360, 7.0) #Pts define diamond
    loc2 = gpdVectorOffset(gpsLat1,gpsLong1, (brng)%360, 7.0)    #Around gps1
    loc3 = gpdVectorOffset(gpsLat1,gpsLong1, (brng-90)%360, 7.0)
    loc4 = (gpsLat2,gpsLon2)
    legQueue.put(loc1[0],loc1[2],xteMin,xteMax)    #Load in the legs
    legQueue.put(loc2[0],loc2[2],-2.0,50.0)        #Different xte reqs for the
    legQueue.put(loc3[0],loc3[2],-2.0,50.0)        #Short legs
    legQueue.put(loc4[0],loc4[2],xteMin,xteMax)
  elif a == "StationKeeping":       #Stay within box created by 4 gps points
    pass#Complicated alg, to define later
  else:
    compMode = "Wait"        #If invalid input, do nothing
    loadLegQueue()

  checkLeg()                        #Get first waypoint and publish
  

def airmar_callback(data):
  global currentLat
  global currentLong
  global apWndSpd
  global apWndDir
  global truWndSpd
  global truWndDir
  global timeSinceLastCheck

  currentLat = data.lat
  currentLong = data.long
  apWndSpd = data.apWndSpd
  apWndDir = data.apWndDir
  truWndSpd = data.truWndSpd
  truWndDir = data.truWndDir
  
  #If we near the end of the leg or sailing a steady angle to the wind,
  #we should call checkLeg very often.  If we
  #are far away, we can poll less frequently
  if(cautious or 
    ((time.time() - timeSinceLastCheck) >= 
      1.0 if compMode == "MaintainPointOfSail" else 5.0)):
    checkLeg()
    timeSinceLastCheck = time.time()


#Every time new competition info comes in, we must re-route everything
def competition_info_callback(data):
  global legQueue
  global compMode
  global gpsLat1
  global gpsLong1
  global gpsLat2
  global gpsLong2
  global gpsLat3
  global gpsLong3
  global gpsLat4
  global gpsLong4
  global angle
  global xteMin
  global xteMax

  legQueue = queue.Queue(maxsize=0)   #Create new leg queue
  #Switch on the competition mode but PYTHON HAS NO SWITCH!
  compMode = data.comp_mode
  gpsLat1 = data.gps_lat1
  gpsLong1 = data.gps_long1
  gpsLat2 = data.gps_lat2
  gpsLong2 = data.gps_long2
  gpsLat3 = data.gps_lat3
  gpsLong3 = data.gps_long3
  gpsLat4 = data.gps_lat4
  gpsLong4 = data.gps_long4
  angle = (data.angle)%360.0   #Angle must be a valid  compass bearing
  xteMin = data.xte_min
  xteMax = data.xte_max

  loadLegQueue()      #Recalculate the leg queue every time we get a new
                      #message from competition_info


#going to be sent from the controller probably (some switch that changes between autonomous and manual)
#def manual_callback(data):
#  global isManual = data.data #the boolean that describes if we are in autonomous or in manual mode.


def listener():
  rospy.init_node("captain")
  rospy.Subscriber("airmar_data", AirmarData, airmar_callback)
  rospy.Subscriber("competition_info", CompetitionInfo, competition_info_callback)
  #rospy.Subscriber("manual_mode", Bool, manual_callback)

  rospy.spin() 

if __name__ == "__main__":
  listener()





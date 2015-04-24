#!/usr/bin/env python
#captain.py                                     Eric Anderson Mar 15
#Implements the captain node as described in the  specs
#Andrew Malta 4/17/15 - fixing things

#TO DO: Implement WAIT and StationKeeping

import time
import roslib
import rospy
import sys
import Queue

from gpsCalc import gpsCalc  #import all the gps functions
from captain.msg import LegInfo
from airmar.msg import AirmarData #TODO partially implemented.
from captain.msg import CompetitionInfo
from std_msgs.msg import Bool #the bool for manual mode

class Airmar():
    #Internal State
    class Waypoint():
        def __init__(self, wlat,wlong,xteMin,xteMax):
            self.wlat = wlat if abs(wlat) <=90.0 else 0.0
            self.wlong = wlong if abs(wlong) <= 180.0 else 0.0
            self.xteMin = xteMini if xteMin <= -1.0 else -100.0
            self.xteMax = xteMax if xteMax >= 1.0 else 100.0

    def __init__(self):
        #====== Default values on initialization ======#
        self.currentLat = 0.0      # From the airmar
        self.currentLong = 0.0
        self.apWndSpd = 0.0
        self.apWndDir = 0.0
        self.truWndSpd = 0.0
        self.truWndDir = 0.0

        self.compMode = "Wait"     # From competition_info
                                   # POSSIBLE VAULES:
                                   # - Wait
                                   # - SailToPoint
                                   # - MaintainHeading
                                   # - MaintainPointOfSail
                                   # - RoundAndReturn
                                   # - StationKeeping


        self.legQueue = Queue.Queue(maxsize=0)  #A queue of waypoints
        self.beginLat = 0.0
        self.beginLong = 0.0
        self.end = None          #The end waypoint for this leg

        self.legArrivalTol = 2.0 #How close do we have to  get to waypoint to have "arrived"
        self.cautious = False  #~~Must be empirically determined, set to 2.0m for now
        self.cautiousDistance = 10.0   #How far away from waypoint to we start being cautious
        self.timeSinceLastCheck = -1 # Used in Airmar callback

    def publish_captain(self):
        rospy.init_node("captain")
        pub_leg = rospy.Publisher("/leg_info", LegInfo, queue_size = 10)
        leg_info = LegInfo()
        #Invariant: end != None (b/c nec called from checkLeg())
        leg_info.begin_lat = beginLat
        leg_info.begin_long = beginLong
        leg_info.leg_course = gpsBearing(beginLat, beginLong, end.wlat, end.wlong)
        leg_info.end_lat = end.wlat
        leg_info.end_long = end.wlong
        leg_info.xte_min = end.xteMin
        leg_info.xte_max = end.xteMax
        pub_leg.publish(leg_info)

    #Updates the state variables if we have completed a leg or rerouted
    def checkLeg(self):
        if not self.compMode == "Wait":
            if self.end == None:         #If no current waypoint
                pass
            elif self.legQueue.empty():
                self.compMode = "Wait"
                self.loadLegQueue()      #Recursive call may result in two publications in
                return              #rapid succession, but this isn't bad I don't think
            else:
                self.beginLat = self.currentLat      #Next leg starts here
                self.beginLong = self.currentLong
                self.end = self.legQueue.get()
                self.publish_captain()
                rospy.loginfo("[captain] Just published leg info!")

            #Fall thru b/c what if next leg is at the same place as this leg?
            d = self.gpsDistance(self.currentLat,self.currentLong,self.end.wlat,self.end.wlong) #Recognize complete leg
            self.cautious = (d < self.cautiousDistance)     # True if we should start polling more quickly

            # Checks if we've hit our target!
            if d < self.legArrivalTol:
                self.end = None

        rospy.loginfo("[captain] Checked leg!")

    #Loads the leg Queue with appropriate legs given competition_info messages
    def loadLegQueue(self):
        self.legQueue = Queue.Queue(maxsize=0)  #Empty the queue
        self.end = None                         #We are starting over
        if self.compMode == "Wait":                 #Stay in the same place so we can get there 
            pass
        elif self.compMode == "SailToPoint":          #Sail to a gps target
            self.legQueue.put(Waypoint(self.gpsLat1,self.gpsLong1,self.xteMin,self.xteMax)) #Insert a gps loc
        elif self.compMode == "MaintainHeading":      #Sail a constant compass direction forever
            loc = gpsVectorOffset(self.currentLat,self.currentLong,self.angle,100000)
            self.leqQueue.put(Waypoint(loc[0],loc[1],self.xteMin,self.xteMax)) #Sail 100km in direction of angle
        elif self.compMode == "MaintainPointOfSail":        #Sail a constant angle to the wind forever
            course = (self.truWndDir + self.angle) %360.0   #The course relative to the wind
            loc = gpsVectorOffset(self.currentLat,self.currentLong,course,100000)
            self.leqQueue.put(Waypoint(loc[0],loc[1],self.xteMin,self.xteMax)) #Sail 100km in direction of angle
        elif self.compMode == "RoundAndReturn":       #Round a mark at gps1 and return to gps2
            brng = gpsBearing(self.currentLat,self.currentLong,self.gpsLat1,self.gpsLong2)
            loc1 = gpsVectorOffset(self.gpsLat1,self.gpsLong1, (brng+90)%360, 7.0) #Pts define diamond
            loc2 = gpsVectorOffset(self.gpsLat1,self.gpsLong1, (brng)%360, 7.0)    #Around gps1
            loc3 = gpsVectorOffset(self.gpsLat1,self.gpsLong1, (brng-90)%360, 7.0)
            loc4 = (self.gpsLat2,self.gpsLong2)
            self.legQueue.put(loc1[0],loc1[1],self.xteMin,self.xteMax)    #Load in the legs
            self.legQueue.put(loc2[0],loc2[1],-2.0,50.0)        #Different xte reqs for the
            self.legQueue.put(loc3[0],loc3[1],-2.0,50.0)        #Short legs
            self.legQueue.put(loc4[0],loc4[1],self.xteMin,self.xteMax)
        elif self.compMode == "StationKeeping":       #Stay within box created by 4 gps points
            pass # Complicated alg, to define later
        else:
            self.compMode = "Wait"        #If invalid input, do nothing
            # self.loadLegQueue() # FIXME: What's the idea of a recursive call here???


    def airmar_callback(self,data):
        self.currentLat = data.lat
        self.currentLong = data.long
        self.apWndSpd = data.apWndSpd
        self.apWndDir = data.apWndDir
        self.truWndSpd = data.truWndSpd
        self.truWndDir = data.truWndDir

    #Every time new competition info comes in, we must re-route everything
    def competition_info_callback(self,data):
        self.legQueue = Queue.Queue(maxsize=0)   #Create new leg queue
        #Switch on the competition mode but PYTHON HAS NO SWITCH!
        compMode = data.comp_mode
        self.gpsLat1 = data.gps_lat1
        self.gpsLong1 = data.gps_long1
        self.gpsLat2 = data.gps_lat2
        self.gpsLong2 = data.gps_long2
        self.gpsLat3 = data.gps_lat3
        self.gpsLong3 = data.gps_long3
        self.gpsLat4 = data.gps_lat4
        self.gpsLong4 = data.gps_long4
        self.angle = (data.angle)%360.0   #Angle must be a valid  compass bearing
        self.xteMin = data.xte_min
        self.xteMax = data.xte_max

        self.loadLegQueue()      #Recalculate the leg queue every time we get a new
        #message from competition_info

    def manual_callback(self,data):
        '''
        Going to be sent from the RC controller probably (some switch that changes between autonomous and manual)
        '''
        self.isManual = data.data #the boolean that describes if we are in autonomous or in manual mode.



    def listener(self):
        rospy.init_node("captain")
        rospy.loginfo("[captain] Subscribing to airmar_data...")
        rospy.Subscriber("airmar_data", AirmarData, self.airmar_callback)
        rospy.loginfo("[captain] Subscribing to competition_info...")
        rospy.Subscriber("competition_info", CompetitionInfo, self.competition_info_callback)
        rospy.loginfo("[captain] All subscribed, captain has started!")
        #rospy.Subscriber("manual_mode", Bool, manual_callback)

        # If we are close to our target, we are in "cautious" mode,
        # which calls for a higher frequency of checking if we've completed it.
        normal_checkleg_rate = rospy.Rate(0.2) # Hz
        cautious_checkleg_rate = rospy.Rate(1) # Hz

        while not rospy.is_shutdown():
            self.checkLeg() # Checks to see if we've arrived at our target yet

            # Note that self.cautious is set by checkLeg()!
            if self.cautious or self.compMode == "MaintainPointOfSail":
                cautious_checkleg_rate.sleep()
            else:
                normal_checkleg_rate.sleep()




if __name__ == "__main__":
    Airmar().listener()

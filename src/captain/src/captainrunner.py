#!/usr/bin/env python
#captain.py                                     Eric Anderson Mar 15
#Implements the captain node as described in the  specs
#Andrew Malta 4/17/15 - fixing things

#TO DO: Implement StationKeeping

import time
import roslib
import rospy
import sys
import Queue

from gpsCalc import gpsCalc      #import all the gps functions
from captain.msg import LegInfo  #Publish to this
from speed_calculator.msg import SpeedStats  #For truWndDir
from captain.msg import CompetitionInfo      #User input
from std_msgs.msg import Bool                #The bool for manual mode
#FIXME need to add an import for the gps node!

class Captain():
    #Internal State
    class Waypoint():
        def __init__(self, wlat,wlong,xteMin,xteMax):
            self.wlat = wlat if abs(wlat) <=90.0 else 0.0
            self.wlong = wlong if abs(wlong) <= 180.0 else 0.0
            self.xteMin = xteMin if xteMin <= -1.0 else -100.0
            self.xteMax = xteMax if xteMax >= 1.0 else 100.0

    def __init__(self):
        #====== Default values on initialization ======#
        self.currentLat = 0.0      # From the airmar
        self.currentLong = 0.0
#        self.apWndSpd = 0.0       #Don't need any of these fields for captain
#        self.apWndDir = 0.0
#        self.truWndSpd = 0.0
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

        #CONSTANTS
        self.legArrivalTol = 2.0 #How close do we have to  get to waypoint to have "arrived"
        self.cautious = False    #~~Must be empirically determined, set to 2.0m for now
        self.cautiousDistance = 30.0   #How far away from waypoint to we start being cautious
        self.timeSinceLastCheck = -1 # Used in Airmar callback

    def publish_captain(self):
        rospy.init_node("captain")
        pub_leg = rospy.Publisher("/leg_info", LegInfo, queue_size = 10)
        leg_info = LegInfo()
        #Invariant: end != None (b/c nec called from checkLeg())
        leg_info.begin_lat = self.beginLat
        leg_info.begin_long = self.beginLong
        leg_info.leg_course = gpsBearing(beginLat, beginLong, end.wlat, end.wlong)
        leg_info.end_lat = end.wlat
        leg_info.end_long = end.wlong
        leg_info.xte_min = end.xteMin
        leg_info.xte_max = end.xteMax
        pub_leg.publish(leg_info)


    #Eric rewrote checkLeg() to eliminate logic bug on May 6 2015
    #Updates the state variables if we have completed a leg or rerouted
    def checkLeg(self):
        if not self.compMode == "Wait":    #If "wait" to have nonzero functionality, this logic block must change
            if self.end == None:           #If no current waypoint
                if self.legQueue.empty():  #If no next waypoint
                    self.compMode = "Wait" #Go into wait mode, exit
                    self.loadLegQueue()
                    return
                else:                      #Move on to next leg, pull waypoint, start leg
                   self.beginLat = self.currentLat       #From where we currently
                   self.beginLong = self.currentLong
                   self.end = self.legQueue.get()
                   self.publish_captain()
                   rospy.loginfo("[captain] Just  published  leg info!")

            else:                          #If we have a current destination 
                d = self.gpsDistance(self.currentLat,self.currentLong,self.end.wlat,self.end.wlong)  #Distance to waypoint
                self.cautious = (d < self.cautiousDistance)                # True if we should start polling more quickly
                if d < self.legArrivalTol:                                 # Have we "arrived"?
                    self.end = None                                        # If so, wipe current waypoint
                    checkLeg()                                             # This recursion is only ever 1 deep
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

    def speed_calculator_callback(self,data):
        self.truWndDir = data.truWndDir       #This is the only field the captain might need

#    def gps_info_callback(self,data):        #Comment in once topic is defined
#        self.currentLat = data.lat
#        self.currentLong = data.long
 
    #Every time new competition info comes in, we must re-route everything
    def competition_info_callback(self,data):
        self.legQueue = Queue.Queue(maxsize=0)   #Create new leg queue
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
        rospy.loginfo("[captain] Subscribing to speed_calculator...")
        rospy.Subscriber("competition_info", SpeedStats, self.speed_calculator_callback)
        rospy.loginfo("[captain] Subscribing to competition_info...")
        rospy.Subscriber("competition_info", CompetitionInfo, self.competition_info_callback)
        rospy.loginfo("[captain] Subscribing to gps_info...")                 #TODO this topic is not yet written 5/16/15
        rospy.Subscriber("gps_info", GpsInfo, self.gps_info_callback)
        #rospy.loginfo("[captain] Subscribing to manual_mode...")                 #TODO this topic is not yet written 5/16/15
        #rospy.Subscriber("manual_mode", Bool, manual_callback)
        rospy.loginfo("[captain] All subscribed, captain has started!")


        # If we are close to our target, we are in "cautious" mode,
        # which calls for a higher frequency of checking if we've completed it.
        normal_checkleg_rate = rospy.Rate(0.25) # Hz  Check every four seconds if far away
        cautious_checkleg_rate = rospy.Rate(2) # Hz  Check every half second if cautious

        #Infinite Loop
        while not rospy.is_shutdown():
            self.checkLeg() # Checks to see if we've arrived at our target yet

            # Note that self.cautious is set by checkLeg()!
            if self.cautious or self.compMode == "MaintainPointOfSail":
                cautious_checkleg_rate.sleep()
            else:
                normal_checkleg_rate.sleep()




if __name__ == "__main__":
    Captain().listener()

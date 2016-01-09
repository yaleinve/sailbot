#!/usr/bin/env python
#captain.py                                     Eric Anderson Mar 15
#Implements the captain node as described in the  specs
#Andrew Malta 4/17/15 - fixing things

import time
import roslib
import rospy
import sys
import Queue

import gpsCalc      #import all the gps functions
from captain.msg import LegInfo  #Publish to this
from captain.msg import CompetitionInfo      #User input
from std_msgs.msg import Bool                #The bool for manual mode
from airmar.msg import AirmarData


class Waypoint():
    def __init__(self, wlat,wlong,xteMin,xteMax):
        self.wlat = wlat if abs(wlat) <=90.0 else 0.0
        self.wlong = wlong if abs(wlong) <= 180.0 else 0.0
        self.xteMin = xteMin if xteMin <= -1.0 else -100.0 #This is the xte tolerance on the way to the waypoint from current destination
        self.xteMax = xteMax if xteMax >= 1.0 else 100.0   #(cont'd) we require at least a 2m wide corridor (the boat is about 2m!)
    def logWaypoint(self):
        rospy.loginfo('[captain] waypoint')
        rospy.loginfo('    lat: ' + str(self.wlat)+ ', long: ' + str(self.wlong))
        rospy.loginfo('    xtemin: ' + str(self.xteMin) + ', xteMax: ' + str(self.xteMax))


class Captain():
    #Internal State

    def __init__(self):
        #====== Default values on initialization ======#
        self.currentLat = 0.0      # From the airmar
        self.currentLong = 0.0
        self.truWndDir = 0.0

        # Our publisher for leg data to the navigator
        self.pub_leg = rospy.Publisher("/leg_info", LegInfo, queue_size = 10)

        self.compMode = "Wait"     # From competition_info
                                   # POSSIBLE VAULES:
                                   # - Wait
                                   # - SailToPoint
                                   # - MaintainHeading
                                   # - MaintainPointOfSail
                                   # - RoundAndReturn
                                   # - StationKeeping


        self.legQueue = Queue.Queue(maxsize=0)  #A queue of waypoints, no max size
        self.beginLat = 0.0
        self.beginLong = 0.0

        #The end waypoint for this leg
        self.current_target_waypoint = -1 # This is the initialization value. `end` ordinarily
                      # takes either None or a Waypoint, which is the latest


        #CONSTANTS
        self.legArrivalTol = 2.0 #How close do we have to  get to waypoint to have "arrived"
        self.cautious = False
        self.cautiousDistance = 30.0   #How far away from waypoint to we start being cautious
        self.timeSinceLastCheck = -1   #Used in Airmar callback

    def publish_captain(self):
        #rospy.loginfo("[captain] I'm in pub!!!!")
        leg_info = LegInfo()
        #Invariant: end != None (b/c nec called from checkLeg())
        leg_info.begin_lat = self.beginLat
        leg_info.begin_long = self.beginLong
        leg_info.leg_course = gpsBearing(self.beginLat, self.beginLong, self.current_target_waypoint.wlat, self.current_target_waypoint.wlong)
        leg_info.end_lat = self.current_target_waypoint.wlat
        leg_info.end_long = self.current_target_waypoint.wlong
        leg_info.xte_min = self.current_target_waypoint.xteMin
        leg_info.xte_max = self.current_target_waypoint.xteMax
        self.pub_leg.publish(leg_info)


    #Updates the state variables if we have completed a leg or rerouted
    def checkLeg(self):
        if not self.compMode == "Wait":    #If "wait" to have nonzero functionality, this logic block must change
             #If no current waypoint OR this is our very first leg for the given competition mode
            if self.current_target_waypoint == None or self.current_target_waypoint == -1:
                rospy.loginfo("[captain] No more waypoints in queue, journey complete!!!")
                if self.legQueue.empty():  #If no next waypoint
                    self.compMode = "Wait" #Go into wait mode, exit
                    self.loadLegQueue()
                    return
                else:                      #Move on to next leg, pull waypoint, start leg
                   rospy.loginfo("[captain] Moving on to next waypoint")
                   self.beginLat = self.currentLat       #From where we currently are
                   self.beginLong = self.currentLong
                   self.current_target_waypoint = self.legQueue.get()
                   self.publish_captain()
                   rospy.loginfo("[captain] Just  published  leg info!")

            else:                          #If we have a waypoint
                d = gpsDistance(self.currentLat,self.currentLong,self.current_target_waypoint.wlat,self.current_target_waypoint.wlong)  #Distance to waypoint
                self.cautious = (d < self.cautiousDistance)                # True if we should start polling more quickly
                if d < self.legArrivalTol:                                 # Have we "arrived"?
                    self.current_target_waypoint = -1                      # If so, wipe current waypoint
                    self.checkLeg()                                        # This recursion is only ever 1 deep
        rospy.loginfo("[captain] Checked leg!")


    #Loads the leg Queue with appropriate legs given competition_info messages
    def loadLegQueue(self):
        self.legQueue = Queue.Queue(maxsize=0)  #Empty the queue
        self.current_target_waypoint = None                         #We are starting over
        if self.compMode == "Wait":                 #Stay in the same place so we can get there
        # TODO: Wait mode should tell the navigator to quit its latest goal,
        # and possibly let out the sheets / turn the rudder so we move as
        # little as possible
            pass
        elif self.compMode == "SailToPoint":          #Sail to a gps target
            #rospy.loginfo("[captain] I'm in SailToPoint!")
            self.legQueue.put(Waypoint(self.gpsLat1,self.gpsLong1,self.xteMin,self.xteMax)) #Insert a gps loc
            #self.legQueue.put(Waypoint(0,0,-43,43))
        elif self.compMode == "MaintainHeading":      #Sail a constant compass direction forever
            #rospy.loginfo('[captain] entered maintain heading')
            #rospy.loginfo('[captain] angle is ' + str(self.angle))
            loc = gpsVectorOffset(self.currentLat,self.currentLong,self.angle,50000)
            #rospy.loginfo('[captain] loc is ' + str(loc[0]) + ' , ' + str(loc[1]))
            self.legQueue.put(Waypoint(loc[0],loc[1],self.xteMin,self.xteMax)) #Sail 100km in direction of angle
        elif self.compMode == "MaintainPointOfSail":        #Sail a constant angle to the wind forever
            course = (self.truWndDir + self.angle) %360.0   #The course relative to the wind
            loc = gpsVectorOffset(self.currentLat,self.currentLong,course,50000)
            self.leqQueue.put(Waypoint(loc[0],loc[1],self.xteMin,self.xteMax)) #Sail 100km in direction of angle
        elif self.compMode == "RoundAndReturn":       #Round a mark at gps1 and return to gps2
            brng = gpsBearing(self.currentLat,self.currentLong,self.gpsLat1,self.gpsLong1)
            loc1 = gpsVectorOffset(self.gpsLat1,self.gpsLong1, (brng+90)%360, 7.0) #Pts define diamond
            loc2 = gpsVectorOffset(self.gpsLat1,self.gpsLong1, (brng)%360, 7.0)    #Around gps1
            loc3 = gpsVectorOffset(self.gpsLat1,self.gpsLong1, (brng-90)%360, 7.0)
            loc4 = (self.gpsLat2,self.gpsLong2)
            w1 = Waypoint(loc1[0],loc1[1],self.xteMin,self.xteMax)
            w1.logWaypoint()
            w2 = Waypoint(loc2[0],loc2[1],-2.0,50.0)
            w2.logWaypoint()
            w3 = Waypoint(loc3[0],loc3[1],-2.0,50.0)
            w3.logWaypoint()
            w4 = Waypoint(loc4[0],loc4[1],self.xteMin,self.xteMax)
            w4.logWaypoint()
            self.legQueue.put(w1)        #Load in the legs
            self.legQueue.put(w2)        #Different xte reqs for the
            self.legQueue.put(w3)        #Short legs
            self.legQueue.put(w4)

        elif self.compMode == "StationKeeping":       #Stay within box created by 4 gps points
            pass # Complicated alg, to define later
        else:
            self.compMode = "Wait"        #If invalid input, do nothing

        self.checkLeg()

    def speed_calculator_callback(self,data):
        self.truWndDir = data.truWndDir       #This is the only field the captain might need

    def gps_info_callback(self,data):        #Comment in once topic is defined
        #topic format defined in sensor_msgs.msg
        self.currentLat = data.latitude
        self.currentLong = data.longitude

    #Every time new competition info comes in, we must re-route everything
    def competition_info_callback(self,data):
        rospy.loginfo("[captain] I'm in the competition_info_callback!")
        self.legQueue = Queue.Queue(maxsize=0)   #Create new leg queue
        self.compMode = data.comp_mode
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
        rospy.Subscriber("/speed_stats", SpeedStats, self.speed_calculator_callback) #FIXME: Find the right name for speed stats node
        rospy.loginfo("[captain] Subscribing to competition_info...")
        rospy.Subscriber("/competition_info", CompetitionInfo, self.competition_info_callback)
        rospy.loginfo("[captain] Subscribing to fix...")                 #TODO this topic is not yet written 5/16/15
        rospy.Subscriber("fix", NavSatFix, self.gps_info_callback)       #TODO ask alex why no slash in topic name!!
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

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

#Waypoint class allows us to easily handle legs in a journey
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

        #Default into wait mode
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
        self.current_target_waypoint = -1


        #CONSTANTS
        self.legArrivalTol = 1.0 #How close do we have to  get to waypoint to have "arrived." 1.0m for now
        self.cautious = False
        self.cautiousDistance = 30.0   #How far away from waypoint to we start being cautious

    def publish_captain(self):
        #rospy.loginfo("[captain] I'm in pub!!!!")
        leg_info = LegInfo()
        #Invariant: end waypoint != None (b/c nec called from checkLeg())
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

    #Loads the leg Queue with appropriate legs given competition_info messages
    #ALGORITHMS
    def loadLegQueue(self):
        self.legQueue = Queue.Queue(maxsize=0)  #Empty the queue
        self.current_target_waypoint = None                         #We are starting over

        #==================================#
        #   DEBUGGING MODES                #
        #==================================#

        #Wait mode
        if self.compMode == "Wait":                 #Stay in the same place so we can get there
        # TODO: Wait mode should tell the navigator to quit its latest goal,
        # and possibly let out the sheets / turn the rudder so we move as
        # little as possible
            pass

        #Sail to a given GPS Point
        elif self.compMode == "SailToPoint":          #Sail to a gps target
            #rospy.loginfo("[captain] I'm in SailToPoint!")
            self.legQueue.put(Waypoint(self.gpsLat1,self.gpsLong1,self.xteMin,self.xteMax)) #Insert a gps loc
            #self.legQueue.put(Waypoint(0,0,-43,43))

        #Sail a compass course
        elif self.compMode == "MaintainHeading":      #Sail a constant compass direction forever
            #rospy.loginfo('[captain] entered maintain heading')
            #rospy.loginfo('[captain] angle is ' + str(self.angle))
            loc = gpsVectorOffset(self.currentLat,self.currentLong,self.angle,50000)
            #rospy.loginfo('[captain] loc is ' + str(loc[0]) + ' , ' + str(loc[1]))
            self.legQueue.put(Waypoint(loc[0],loc[1],self.xteMin,self.xteMax)) #Sail 50km in direction of angle

        #Sail relative to the wind    TODO: Should we update this if the wind changes?
        elif self.compMode == "MaintainPointOfSail":        #Sail a constant angle to the wind forever
            course = (self.truWndDir + self.angle) %360.0   #The course relative to the wind
            loc = gpsVectorOffset(self.currentLat,self.currentLong,course,50000)
            self.leqQueue.put(Waypoint(loc[0],loc[1],self.xteMin,self.xteMax)) #Sail 100km in direction of angle

        #==================================#
        #   COMPETITION MODES              #
        #==================================#

        ####  NAVIGATION TEST  #########
        # Sail through a start line, round a mark to port, sail back down through start line
        # Wdwd mark is approx 60m upwind
        # Competition ends when cross finish line (or extensions).  Penalties dep on how where on extension you cross
        # Line is 3m wide (!!)
        # Human input: gps 1 is wdwd mark, gps 2 is left start mark (the pin), gps 3 is right start mark (the boat)
        # Alg sketch: start, sail diamond around mark, sail to x meters above line, sail straight down through

        elif self.compMode == "RoundAndReturn":       #Round a mark at gps1 and return to gps2
            brng = gpsBearing(self.currentLat,self.currentLong,self.gpsLat1,self.gpsLong1)
            #Diamond around mark 1
            loc1 = gpsVectorOffset(self.gpsLat1,self.gpsLong1, (brng+90)%360, 7.0) #Pts define diamond
            loc2 = gpsVectorOffset(self.gpsLat1,self.gpsLong1, (brng)%360, 7.0)    #Around gps1
            loc3 = gpsVectorOffset(self.gpsLat1,self.gpsLong1, (brng-90)%360, 7.0)
            #Explicitly cross the line perpendicularly
            #First, calculate center of line:
            lToRBearing = gpsBearing(self.gpsLat2,self.gpsLong2, self.gpsLat3,self.gpsLong3)
            lToRDistance = gpsDistance(self.gpsLat2,self.gpsLong2, self.gpsLat3,self.gpsLong3)
            centerLoc = gpsVectorOffset(self.gpsLat2,self.gpsLong2,lToRBearing, lToRDistance/2)
            loc4 = gpsVectorOffset(centerLoc[0],centerLoc[1], (lToRBearing-90)%360, 5.0)
            loc5 = gpsVectorOffset(centerLoc[0],centerLoc[1], (lToRBearing+90)%360, 5.0)

            w1 = Waypoint(loc1[0],loc1[1],self.xteMin,self.xteMax)
            w1.logWaypoint()
            w2 = Waypoint(loc2[0],loc2[1],-2.0,50.0)
            w2.logWaypoint()
            w3 = Waypoint(loc3[0],loc3[1],-2.0,50.0)
            w3.logWaypoint()
            w4 = Waypoint(loc4[0],loc4[1],self.xteMin,self.xteMax)
            w4.logWaypoint()
            w5 = Waypoint(loc5[0],loc5[1],-1,1)  #The final leg has very low xte tolerance so we actually cross the correct line
            w5.logWaypoint()
            self.legQueue.put(w1)        #Load in the legs
            self.legQueue.put(w2)        #Different xte reqs for the
            self.legQueue.put(w3)        #Short legs
            self.legQueue.put(w4)
            self.legQueue.put(w5)

        #####   STATION KEEPING #########

        elif self.compMode == "StationKeeping":       #Stay within box created by 4 gps points
            pass # Complicated alg, to define later

        #####  WAIT   ###################

        else:
            self.compMode = "Wait"        #If invalid input, do nothing

        self.checkLeg()


    def airmar_callback(self,data):
        self.truWndDir = data.truWndDir
        self.currentLat = data.lat
        self.currentLong = data.long

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

    def listener(self):
        rospy.init_node("captain")
        rospy.loginfo("[captain] Subscribing to competition_info...")
        rospy.Subscriber("/competition_info", CompetitionInfo, self.competition_info_callback)
        rospy.loginfo("[captain] Subscribing to airmar_data...")
        rospy.Subscriber("/airmar_data", AirmarData, self.airmar_callback)

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

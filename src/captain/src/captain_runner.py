#!/usr/bin/env python
#captain.py                                     Eric Anderson Mar 15
#Implements the captain node as described in the  specs
#Andrew Malta 4/17/15 - fixing things

import time
import roslib
import rospy
import sys
import Queue
import mraa
import math

from compassCalc import *
from gpsCalc import *    #import all the gps functions
from captain.msg import LegInfo  #Publish to this
from captain.msg import CompetitionInfo      #User input
from captain.msg import AutonomousStatus
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
        self.currentHeading = 0.0

        #Autonomous pins from RC
        self.autonomousPin = mraa.Aio(1)  #The analog pin number for aux 1
        self.auxVoltDivide = 400          #TODO: empirically measure this!
        self.currentlyAutonomous = False  #Default into manual on boot (so if in manual during startup we don't lose control)

        #Relay pins
        self.sailRelayPin =  mraa.Gpio(4)    #Pin 4 is sail relays
        self.sailRelayPin.dir(mraa.DIR_OUT)
        self.rudderRelayPin = mraa.Gpio(2)   #Pin 2 is rudder relay
        self.rudderRelayPin.dir(mraa.DIR_OUT)


        # Our publisher for leg data to the navigator
        self.pub_leg = rospy.Publisher("/leg_info", LegInfo, queue_size = 10)
        self.pub_autonomous = rospy.Publisher("/autonomous_status", AutonomousStatus, queue_size = 10)

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
        self.taskBeginTime = -1.0    #For timed legs...

        #The end waypoint for this leg
        self.current_target_waypoint = -1


        #CONSTANTS
        self.legArrivalTol = 1.0       #How close do we have to  get to waypoint to have "arrived." 1.0m for now
        self.cautious = False
        self.cautiousDistance = 30.0   #How far away from waypoint to we start being cautious

    def publish_captain(self):
        #rospy.loginfo("[captain] I'm in pub!!!!")
        if self.current_target_waypoint == None or self.current_target_waypoint == -1:
            return
        leg_info = LegInfo()
        leg_info.begin_lat = self.beginLat
        leg_info.begin_long = self.beginLong
        leg_info.leg_course = gpsBearing(self.beginLat, self.beginLong, self.current_target_waypoint.wlat, self.current_target_waypoint.wlong)
        leg_info.end_lat = self.current_target_waypoint.wlat
        leg_info.end_long = self.current_target_waypoint.wlong
        leg_info.xte_min = self.current_target_waypoint.xteMin
        leg_info.xte_max = self.current_target_waypoint.xteMax
        self.pub_leg.publish(leg_info)

    #Publish our autonomous/manual state
    def publish_autonomous(self):
        msg = AutonomousStatus()
        msg.currentlyAutonomous = self.currentlyAutonomous
        self.pub_autonomous.publish(msg)

    #Do some checks and pull the next waypoint out of the queue
    def getNextWaypoint(self):
        if self.legQueue.empty():  #If no next waypoint
            rospy.loginfo("[captain] No more waypoints in queue, journey complete!!!")
            self.compMode = "Wait" #Go into wait mode, exit
            self.loadLegQueue()
        else:                      #Move on to next leg, pull waypoint, start leg
            rospy.loginfo("[captain] Popping Waypoint from Queue")
            self.beginLat = self.currentLat       #From where we currently are
            self.beginLong = self.currentLong
            self.current_target_waypoint = self.legQueue.get()
            self.publish_captain()
            rospy.loginfo("[captain] Just  published  leg info!")




    '''
    Updates the state variables if we have completed a leg or rerouted
    Note that, even though loadLegQueue can call checkLeg and checkLeg can call loadLegQueue,
    we shouldn't be at risk of infinite recursion based on the logic (mode in checkLeg is set to
    wait if loadLegQueue is called, which means the next checkLeg call is a no-op
    '''
    def checkLeg(self):
        stationKeepingTaskDuration = 5 #300  #Five minutes in the box

        #============================#
        #  MODE SPECIFIC CHECKS      #
        #============================#

        #Skip everything for wait   #TODO: change this?
        if self.compMode == "Wait":
            return

        if self.compMode == "StationKeeping":
            #Time to exit the box?
            if self.taskBeginTime > 0.0 and rospy.get_time() - self.taskBeginTime >= stationKeepingTaskDuration:
                #TODO: improve this really crappy "sail out of the box by going straight" algorithm
                self.compMode = "MaintainHeading"
                self.angle = self.currentHeading
                self.loadLegQueue()
                return                     #Return here, simplifies control flow (no deep recursion with nearby waypoints), just let main loop run again

        #============================#
        #  GENERIC CHECK             #
        #============================#

        #Do we need a new waypoint?
        if self.current_target_waypoint == None or self.current_target_waypoint == -1:
            self.getNextWaypoint()
            return
        #If we have a waypoint, how far away are we from it?
        else:
            d = gpsDistance(self.currentLat,self.currentLong,self.current_target_waypoint.wlat,self.current_target_waypoint.wlong)  #Distance to waypoint
            #rospy.loginfo("[captain] in checkLeg, distance to waypoint calc'ed as : " + str(d))
            #rospy.loginfo("[captain] legArrival Tol is " + str(self.legArrivalTol))
            #rospy.loginfo("[captain] d < tol: " + str(d < self.legArrivalTol))
            self.cautious = (d < self.cautiousDistance)                # True if we should start polling more quickly
            if d < self.legArrivalTol:                                 # Have we "arrived"?
                self.current_target_waypoint = -1                      # If so, wipe current waypoint
                self.checkLeg()                                        # This recursion is only ever 3 deep (checkleg, loadLegQueue, checkleg (which is now in wait mode) )



    #Loads the leg Queue with appropriate legs given competition_info messages
    # TASK ALGORITHMS GO IN HERE!!!!!
    def loadLegQueue(self):
        self.legQueue = Queue.Queue(maxsize=0)  #Empty the queue
        self.current_target_waypoint = None                         #We are starting over

        #==================================#
        #   DEBUGGING / BASIC MODES        #
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
        #40m square box, square to the wind.  Stay in the box for five minutes, then exit the box
        #Input: lat/long of box corners in following order: nw, sw, se, ne (relative to wind)
        #Alg sketch: sail a narrow corridor to the middle of the box laterally, 2/3 upwind
        elif self.compMode == "StationKeeping":       #Stay within box created by 4 gps points
            #Bearing west to east in box
            wToEBrng = (gpsBearing(self.gpsLat1, self.gpsLong1,self.gpsLat4,self.gpsLong4) +  gpsBearing(self.gpsLat2, self.gpsLong2,self.gpsLat3,self.gpsLong3))/2.0
            sToNBrng = (gpsBearing(self.gpsLat2, self.gpsLong2,self.gpsLat1,self.gpsLong1) +  gpsBearing(self.gpsLat3, self.gpsLong3,self.gpsLat4,self.gpsLong4))/2.0
            center = gpsVectorOffset(self.gpsLat1,self.gpsLong1, (wToEBrng - compass_diff(wToEBrng,sToNBrng)/2)%360 , math.sqrt(2*40*40)/2)  #Follow a diagonal, more or less
            #West location
            loc1 = gpsVectorOffset(center[0],center[1],(wToEBrng-180)%360, 12)
            #East location
            loc2 = gpsVectorOffset(center[0],center[1],wToEBrng,12)
            #Add these locations to the queue over and over and over
            w_center = Waypoint(center[0], center[1], -5.0, 5.0)       #We're going to the center, first, so we don't mess up at the beginning
            w_center.logWaypoint()
            w1 = Waypoint(loc1[0],loc1[1],self.xteMin,self.xteMax)
            w1.logWaypoint()
            w2 = Waypoint(loc2[0],loc2[1],self.xteMin,self.xteMax)
            w2.logWaypoint()
            #Add endless queue of back and forth after one center point
            self.legQueue.put(w_center)
            for i in range(1000):
                self.legQueue.put(w1)
                self.legQueue.put(w2)

        #####  ILLEGAL INPUT  ###################

        else:
            self.compMode = "Wait"        #If invalid input, do nothing

       #Let's kick things off!
        self.checkLeg()


    #Reads the aux inputs, switches relays and republishes leg info accordingly
    #Note that thi may call publish_captain() before a waypoint has been pulled off the queue,
    #So publish_captain has to handle that case
    def checkAutonomous(self):
        #TODO: might want to switch these- consider case when controller is turned off.  We want to default to autonomous, right?
        #Manual Mode
        if (self.autonomousPin.read() < self.auxVoltDivide):
            #If we just switched modes
            if self.currentlyAutonomous == True:
                rospy.loginfo('[captain] switching into manual mode')
                self.currentlyAutonomous = False
                #self.rudderRelayPin.write(0)
                #self.sailRelayPin.write(0)
                self.publish_autonomous()
        #Autonomous mode
        else:
            #If we just switched modes
            if self.currentlyAutonomous == False:
                rospy.loginfo('[captain] Switching into autonomous mode')
                self.currentlyAutonomous = True
                #self.rudderRelayPin.write(1)
                #self.sailRelayPin.write(1)
                self.publish_autonomous()

                #For station keeping, we'll start the five minute timer when we go into autonomous mode
                if self.compMode == 'StationKeeping':
                    self.taskBeginTime = rospy.get_time()
                    rospy.loginfo('[captain] StationKeeping start time recorded as : ' + str(self.taskBeginTime))

    def airmar_callback(self,data):
        self.truWndDir = data.truWndDir
        self.currentLat = data.lat
        self.currentLong = data.long
        self.currentHeading = data.heading
        rospy.loginfo("airmar callback- lat: " + str(self.currentLat) +", long: "  + str(self.currentLong))

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
            self.checkLeg()  # Checks to see if we've arrived at our target yet
            self.checkAutonomous()  #Because this is called immediately after checkLeg, publishing invariant is maintained
            # Note that self.cautious is set by checkLeg()!
            if self.cautious or self.compMode == "MaintainPointOfSail" or self.compMode == "MaintainHeading":
                cautious_checkleg_rate.sleep()
            else:
                normal_checkleg_rate.sleep()




if __name__ == "__main__":
    Captain().listener()

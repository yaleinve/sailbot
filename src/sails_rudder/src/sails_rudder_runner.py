#!/usr/bin/env python2.7
# Acshi Haggenmiller 11/15
# Sail and rudder controller originally developed in a simulated sailing environment (sailingsimulator.go)
# Largely PD control of rudder and PI control of sails to steer the boat
# Rudder control becomes PID if needed to achieve straight sailing
# All values are represented as angles in degrees
#Implements a bare bones sail controller that cues off the airmar
#and linearly maps feasible sailing directions to feasible sail positions.
#Because of how our servo drivers work, we can use an abstract sail trim scale
#here from 0 (max trim) to 100 (max ease)

#Basic Imports
import roslib
import rospy
import sys
import time
import math

from compassCalc import *

#Import message types
from airmar.msg import AirmarData #Read from
from tactics.msg import NavTargets
from sails_rudder.msg import SailsRudderPos, SailsRudderStatus #Write to

# linear interpolation of x in range from minX to maxX
# onto the range minVal to maxVal.
# x is coerced into the range minX to maxX if it isn't already.
def lerp(x, minX, maxX, minVal, maxVal):
    x = min(max(x, minX), maxX)
    return (x - minX) / (maxX - minX) * (maxVal - minVal) + minVal

# coerses the angle into the given range, using modular 360 arithmetic
# If min angle to max angle is 360 degrees, then only modular arithmetic occurs
# if min angle and max angle do not span 360 degrees, then the value is coerced
# to the closer of the two angles.
# angleToRange(0, 360, 720) = 360
# angleToRange(-40, 0, 360) = 320
# angleToRange(20, 100, 140) = 100
def angleToRange(a, minA, maxA):
    a = math.fmod(a, 360)
    modMinA = math.fmod(minA, 360)
    modMaxA = math.fmod(maxA, 360)
    if modMinA >= modMaxA:
        modMaxA += 360

    if a < modMinA and (modMinA - a > a + 360 - modMaxA):
        a += 360
    elif a > modMaxA and (a - modMaxA > modMinA - (a - 360)):
        a -= 360
    a = min(max(a, modMinA), modMaxA)

    # Now return a to be between original min and max
    if a < minA:
        a += math.trunc((maxA - a) / 360) * 360
    elif a > maxA:
        a -= math.trunc((a - minA) / 360) * 360
    return a

def normalizeAngle(a):
    while a <= -180:
        a += 360
    while a > 180:
        a -= 360
    return a

class SailsRudder():
    def __init__(self):
        self.pub_sailsRudderPos = rospy.Publisher("/sails_rudder_pos", SailsRudderPos, queue_size = 10) #Set up our publisher
        self.pub_sailsRudderStatus = rospy.Publisher("/sails_rudder_status", SailsRudderStatus, queue_size = 10) #Set up our publisher
        self.reqedMain = 90.0 #"Requested" positions i.e. those that were last published
        self.reqedJib = 90.0 #Init is fully eased sails (avoids mechanical breakage?)
        self.reqedRudder = 0.0

        # Whether we are on the first cycle, and should publish positions no matter what
        self.firstCycle = True

        # Previously calculated, but not necessarily requested positions
        self.mainPos = 90.0
        self.jibPos = 90.0
        self.rudderPos = 0.0

        #CONSTANTS
        self.mainTolerance = 5.0 #The differences at which point we republish
        self.jibTolerance = 5.0
        self.rudderTolerance = 1.0

        # The lowest off wind angle we will sail
        self.minPointingAngle = 50.0 #THIS IS THE HARDCODED POINTING ANGLE IN TACTICS!  IT MUST BE ADJUSTED IF THE VALUE IN TACTICS IS CHANGED!!!
        # The angle at or above which we put both sails all the way out
        self.runningAngle = 165.0 #THIS IS  " " " " RUNNING ANGLE " " " " " "

        self.maxSailEase = 90.0 # Angle from straight back to max ease of main (and for now, jib)
        self.maxTurnOffset = 25.0 # The angle away from extremes that maximizes turning with the sails
        self.rudderRange = 50.0 # The maximum angle the rudder can be turned in either direction off straight
        self.highTurnAngle = 30.0 # The angle against the negative velocity which will maximize turning, rough model of stall angle
        self.minClosedAngle = 15.0 # Keep the jib open by at least this angle

        # PID control of rudder for course correction
        self.lastUpdateTime = 0 # time.clock() value used to time PID loop
        self.previousInput = 0.0
        # control coefficients, optimized in simulation, hopefully just a constant factor off from what would be good for the real Racht
        self.rudderP = 7.94447
        self.rudderI = 0.25778
        self.rudderD = 3.83347
        self.rudderITerm = 0.0
        self.sailP = 8.90936
        self.sailI = 0.39767
        self.sailITerm = 0.0

        self.rudderIDecay = 0.95 # decay factor per second of rudder integral when sails could contribute more

        # Input values from other nodes
        self.targetHeading = 0.0
        self.boatHeading = 0.0
        self.velocityDirection = 0.0
        self.apparentWindDirection = 0.0
        self.trueWindDirection = 0.0

    def publish_positions(self):
        # rospy.loginfo("Publishing")
        sailsRudderPos = SailsRudderPos()
        sailsRudderPos.mainPos = self.reqedMain
        sailsRudderPos.jibPos = self.reqedJib
        sailsRudderPos.rudderPos = self.reqedRudder
        self.pub_sailsRudderPos.publish(sailsRudderPos)

    def tactics_callback(self, data):
        self.targetHeading = data.targetHeading
        self.updatePositions()

    def airmar_callback(self, data):
        self.apparentWindDirection = data.apWndDir
        self.trueWindDirection = data.truWndDir
        self.velocityDirection = data.cog # course over ground
        self.boatHeading = data.heading
        # self.updatePositions()

    def updatePositions(self):
        status = SailsRudderStatus()

        ################################
        #    RUDDER CONTROL            #
        ################################

        # Use straight back from the back of the boat as a reference point 0 for the rudder angle
        # So a positive angle results in a left turn from the rudder, assuming forward motion

        # Do not even attempt to sail into the wind
        if abs(normalizeAngle(self.targetHeading - self.trueWindDirection)) < self.minPointingAngle:
            rospy.logwarn("Attempting to sail at impossible target heading: " + str(normalizeAngle(self.targetHeading - self.apparentWindDirection)))
            return

        # Calculate the limits of PID output values

        # Use the lower-bound of the rudder as a reference point 0.
        # So an angle here of 0 would correspond to the rudder being rudderRange
        # degrees counterclockwise from straight.

        # The angles of the rudder most effective at effecting rotation
        # are those that are at highTurnAngle to the velocity of the boat
        # n.b. velocityDirection is cog (course over ground)
        leeway = normalizeAngle(self.velocityDirection - self.boatHeading)
        status.leeway = leeway

        # The three cases below are that 1. there are good positions for the rudder to turn
        # 2. the boat is moving backwards, so the rudder can go the opposite direction
        # 3. no matter how the rudder is positioned, the boat will be turned and we don't
        # have rudder control, so just keep the rudder neutral.
        # Calculate maximum and minimum PID output values currently realizable with our rudder
        # This varies over time as the boat's orientation and velocity and the wind vary
        # Note that PID values are negative to indicate left turning, and positive for right
        # But the rudder direction angles depend also on the leeway
        if abs(leeway) <= self.rudderRange:
            status.navcase = 1
            optimalTurnLeft = leeway + self.highTurnAngle
            optimalTurnRight = leeway - self.highTurnAngle
            turnLeftDirection = min(optimalTurnLeft, self.rudderRange)
            turnRightDirection = max(optimalTurnRight, -self.rudderRange)
            pidLeftLimit = lerp(turnLeftDirection, optimalTurnRight, optimalTurnLeft, 100, -100)
            pidRightLimit = lerp(turnRightDirection, optimalTurnRight, optimalTurnLeft, 100, -100)
        elif abs(leeway) >= 180 - self.rudderRange:
            status.navcase = 2
            oppositeTurnCenter = normalizeAngle(self.velocityDirection - (self.boatHeading + 180))
            optimalTurnLeft = oppositeTurnCenter - self.highTurnAngle
            optimalTurnRight = oppositeTurnCenter + self.highTurnAngle
            turnLeftDirection = max(optimalTurnLeft, -self.rudderRange)
            turnRightDirection = min(optimalTurnRight, self.rudderRange)
            pidLeftLimit = lerp(turnLeftDirection, optimalTurnLeft, optimalTurnRight, -100, 100)
            pidRightLimit = lerp(turnRightDirection, optimalTurnLeft, optimalTurnRight, -100, 100)
        else:
            status.navcase = 3
            optimalTurnLeft = 0.0
            optimalTurnRight = 0.0
            turnLeftDirection = 0.0
            turnRightDirection = 0.0
            pidLeftLimit = 0.0
            pidRightLimit = 0.0

        # Adjust rudder to get to desired heading
        # Rudder needs to be off from the boat velocity to generate
        # torque needed to turn the boat

        # the PID loop needs time statistics for the I and D terms.
        timeNow = time.time()
        controlInterval = timeNow - self.lastUpdateTime
        self.lastUpdateTime = timeNow

        # PID control
        pidSetpoint = self.targetHeading
        # simulate some error in our ability to read our heading
        pidInput = self.boatHeading

        courseError = angleToRange(pidSetpoint - pidInput, -180, 180)
        status.err = courseError

        # Make adding to the rudder integral term conditional on the sails integral
        # term being maxed out. That gives preference to the sails fixing this kind of thing.
        # However, to prevent impulses, we allow changes that would decrease magnitude
        # and also cause slow decay when the sails could contribute more
        # Integral term for rudder can only go up when sails are maxed out, but can go down any time
        # sail I term maxed out
        if abs(self.sailITerm) >= 95.0 or \
           (self.rudderITerm > 0.0 and courseError < 0.0 or self.rudderITerm < 0.0 and courseError > 0.0): #If rudderITerm would decrease in simple PID alg
            self.rudderITerm += courseError * self.rudderI * controlInterval
        else:
            # Allow the integral to smoothly decay
            self.rudderITerm *= pow(self.rudderIDecay, controlInterval)

        # Prevent the integral term from exceeding the limits of the PID output
        if self.rudderITerm >= pidRightLimit or self.rudderITerm <= pidLeftLimit:
            self.rudderITerm = max(min(self.rudderITerm, pidRightLimit), pidLeftLimit)

        inputDerivative = angleToRange(pidInput - self.previousInput, -180, 180)
        pidOutputValue = courseError * self.rudderP + self.rudderITerm - inputDerivative * self.rudderD / controlInterval
        self.previousInput = pidInput

        status.rudderPCont = courseError * self.rudderP
        status.rudderICont = self.rudderITerm
        status.rudderDCont = -inputDerivative * self.rudderD / controlInterval

        # conversion of that output value (from -100 to 100) to a physical rudder orientation
        constrainedPidValue = max(min(pidOutputValue, pidRightLimit), pidLeftLimit)
        newRudderPos = lerp(constrainedPidValue, -100, 100, optimalTurnLeft, optimalTurnRight)

        # Smoothen out the change to the rudder
        self.rudderPos = lerp(4.0 * controlInterval, 0, 1, self.rudderPos, newRudderPos)

        ################################
        #    SAIL CONTROL              #
        ################################
        # the apparent wind direction is already relative to the boat heading. (relative to the physical sensor, that is)
        # make sails maximize force. See points of sail for reference.
        offWindAngle = angleToRange(self.apparentWindDirection, -180, 180)
        newMainPos = lerp(abs(offWindAngle), self.minPointingAngle, self.runningAngle, 0.0, 90.0)  #A first (linear) approximation of where the sails should go, note right now it's always positive
        newJibPos = newMainPos

        # correct sign- coordinate frame is same as rudder (+ is sails on port side)
        if offWindAngle < 0.0:
            newMainPos = -newMainPos
            newJibPos = -newJibPos

        # Use the sails to help us steer to our desired course
        # Turn to the wind by pulling the mainsail in and freeing the jib
        # Turn away by pulling the jib in and freeing the mailsail

        # Make the sign of course error associate negative with away from wind, and positive into wind
        # This lets the control loop evenly progress back and forth
        sailCourseError = courseError

        if offWindAngle < 0.0:   #We're on starboard
            sailCourseError = -sailCourseError

        # PI control
        sailPTerm = sailCourseError * self.sailP
        self.sailITerm += sailCourseError * self.sailI * controlInterval

        # Prevent value from exceeding the maximum effect value at 100 (prevent integral windup)
        self.sailITerm = max(min(self.sailITerm, 100), -100)
        turnValue = sailPTerm + self.sailITerm

        # The positions that given maximum turning torque from sails
        if newMainPos > 0:
            maxOpenPose = min(newMainPos + self.maxTurnOffset, self.maxSailEase)
            maxClosePose = max(newMainPos - self.maxTurnOffset, 0)
        else:
            maxOpenPose = max(newMainPos - self.maxTurnOffset, -self.maxSailEase)
            maxClosePose = min(newMainPos + self.maxTurnOffset, 0)

        if turnValue > 0.0:
            # Sail more towards the wind
            newMainPos = lerp(turnValue, 0, 100, newMainPos, maxClosePose)
            newJibPos = lerp(turnValue, 0, 100, newJibPos, maxOpenPose)
        else:
            # Sail more away from the wind
            newMainPos = lerp(-turnValue, 0, 100, newMainPos, maxOpenPose)
            newJibPos = lerp(-turnValue, 0, 100, newJibPos, maxClosePose)

        # To maximize flow around the sails, we keep the jib open at least some
        if abs(newJibPos) < self.minClosedAngle:
            newJibPos = -self.minClosedAngle if newJibPos < 0 else self.minClosedAngle

        # Now that calculations are done, since we can't control which way the sails go,
        # we only publish positive values for them
        newMainPos = abs(newMainPos)
        newJibPos = abs(newJibPos)

        # Smoothen out the change to the sails
        self.mainPos = lerp(4.0 * controlInterval, 0, 1, self.mainPos, newMainPos)
        self.jibPos = lerp(4.0 * controlInterval, 0, 1, self.jibPos, newJibPos)

        # Publish every time for testing
        # self.reqedMain = self.mainPos
        # self.reqedJib = self.jibPos
        # self.reqedRudder = self.rudderPos
        # self.publish_positions()

        self.pub_sailsRudderStatus.publish(status)

        #Is it a big enough change that we should republish?  This reduces servo jitters
        if (abs(self.mainPos - self.reqedMain) > self.mainTolerance or
            abs(self.jibPos - self.reqedJib) > self.jibTolerance or
            abs(self.rudderPos - self.reqedRudder) > self.rudderTolerance or
            self.firstCycle):
          self.reqedMain = self.mainPos
          self.reqedJib = self.jibPos
          self.reqedRudder = self.rudderPos
          self.firstCycle = False
          self.publish_positions() #Actually publish the request

    def listener(self):
        rospy.init_node("sailsRudder")
        rospy.Subscriber("/airmar_data", AirmarData, self.airmar_callback)
        rospy.Subscriber("/nav_targets", NavTargets, self.tactics_callback)
        rospy.loginfo("[sails_rudder] All subscribed, sails_rudder has started!")
        rospy.spin()

if __name__ == "__main__":
    SailsRudder().listener()

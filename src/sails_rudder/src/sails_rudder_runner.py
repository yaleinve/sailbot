#!/usr/bin/env python
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

#Import message types
from airmar.msg import AirmarData #Read from
from tactics.msg import TargetHeading
from sails_rudder.msg import SailsRudderPos #Write to

# linear interpolation of x in range from minX to maxX
# onto the range minVal to maxVal.
# x is coerced into the range minX to maxX if it isn't already.
def lerp(x, minX, maxX, minVal, maxVal):
    x = min(max(x, minX), maxX)
    return (x - minX) / (maxX - minX) * (maxVal - minVal) + minVal
    
# coerses the angle into the given range, using modular 360 arithmetic
def coerceAngleToRange(a, minA, maxA):
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

class SailsRudder():
    def __init__(self):
        self.pub_sailsRudderPos = rospy.Publisher("/sails_rudder_pos", SailsRudderPos, queue_size = 10) #Set up our publisher
        self.reqedMain = 90.0 #"Requested" positions i.e. those that were last published
        self.reqedJib = 90.0 #Init is fully eased sails (avoids mechanical breakage?)
        self.reqedRudder = 0.0

        #CONSTANTS
        self.mainTolerance = 5.0 #The differences at which point we republish
        self.jibTolerance = 5.0
        self.rudderTolerance = 1.0
        
        # The lowest off wind angle we will sail
        self.minPointingAngle = 50.0 #THIS IS THE HARDCODED POINTING ANGLE IN TACTICS!  IT MUST BE ADJUSTED IF THE VALUE IN TACTICS IS CHANGED!!!
        # The angle at or above which we put both sails all the way out
        self.runningAngle = 165.0 #THIS IS  " " " " RUNNING ANGLE " " " " " "

        self.maxTurnOffset = 25.0 # The angle away from extremes that maximizes turning with the sails
        self.rudderRange = 60.0 # The maximum angle the rudder can be turned in either direction off straight
        self.highTurnAngle = 30.0 # The angle against the velocity which will maximize turning
        self.minClosedAngle = 25.0 # If both main and jib are closed more than this, jib is held at this angle

        # PID control of rudder for course correction
        self.controlInterval = 0.1 # time in seconds between updates to rudder/sail headings
        self.previousInput = 0.0
        # control coefficients, these may be a good start, but need to be empiracally determined for Racht
        self.rudderP = 1.0
        self.rudderI = 0.4
        self.rudderD = 0.8
        self.rudderITerm = 0.0
        self.sailP = 1.2
        self.sailI = 1.2
        self.sailITerm = 0.0
        
        # Input values from other nodes
        self.targetHeading = 0.0
        self.boatHeading = 0.0
        self.velocityDirection = 0.0
        self.apparentWindDirection = 0.0
        self.trueWindDirection = 0.0

    def publish_positions(self):
        sailsRudderPos = SailsRudderPos()
        sailsRudderPos.mainPos = self.reqedMain
        sailsRudderPos.jibPos = self.reqedJib
        sailsRudderPos.rudderPos = self.reqedRudder
        self.pub_sailsRudderPos.publish(sailsRudderPos)

    def tactics_callback(self, data):
        self.targetHeading = data.targetHeading
        self.updatePositions()
        
    def airmar_callback(self, data):
        #TODO: is airmar apWndDir relative to the boat or absolute?
        #If Absolute:
        #windAngle = compass_diff(data.apWndDir, data.heading)
        #If relative
        self.apparentWindDirection = data.apWndDir
        self.trueWindDirection = data.truWndDir
        self.velocityDirection = data.cog # compass over ground
        self.boatHeading = data.heading
        self.updatePositions()
        
    def updatePositions(self):
        # Calculate the limits of PID output values
        
        # Use the lower-bound of the rudder as a reference point 0.
        # So an angle here of 0 would correspond to the rudder being rudderRange
        # degrees counterclockwise from straight.
        
        # The angles of the rudder most effective at effecting rotation
        # are those that are at highTurnAngle to the velocity of the boat
        optimalTurnLeft = coerceAngleToRange((self.velocityDirection - self.highTurnAngle) - (self.boatHeading - 180) + self.rudderRange, 0, 360)
        optimalTurnRight = coerceAngleToRange((self.velocityDirection + self.highTurnAngle) - (self.boatHeading + 180) + self.rudderRange, -360, 0)
        turningCenter = (optimalTurnLeft + optimalTurnRight) / 2.0
        
        turnLeftDirection = max(coerceAngleToRange(optimalTurnLeft, 0, 2 * self.rudderRange), turningCenter)
        turnLeftDirection = coerceAngleToRange(turnLeftDirection, 0, 2 * self.rudderRange)
        turnRightDirection = min(coerceAngleToRange(optimalTurnRight, 0, 2 * self.rudderRange), turningCenter)
        turnRightDirection = coerceAngleToRange(turnRightDirection, 0, 2 * self.rudderRange)
        
        # Calculate maximum and minimum PID output values currently realizable with our rudder
        # This varies over time as the boat's orientation and velocity and the wind vary
        # Note that positive PID values are negative to indicate to turn left, and positive for right
        # But the rudder direction angles are high(positive) for left and low(negative) for right
        pidLeftLimit = lerp(turnLeftDirection, 0.0, 2 * self.rudderRange, 100, -100)
        pidRightLimit = lerp(turnRightDirection, 0.0, 2 * self.rudderRange, 100, -100)
        
        # Adjust rudder to get to desired heading
        # Rudder needs to be off from the boat velocity to generate
        # torque needed to turn the boat
        
        # PID control            
        pidSetpoint = self.targetHeading
        # simulate some error in our ability to read our heading
        pidInput = self.boatHeading
        
        courseError = coerceAngleToRange(pidSetpoint - pidInput, -180, 180)
        
        # Make the rudder integral term conditional on the sails integral
        # term already being maxed out. That gives preference to the sails
        # fixing this kind of thing.
        if abs(self.sailITerm) == 100.0:
            self.rudderITerm += courseError * self.rudderI  * self.controlInterval
        else:
            self.rudderITerm = 0.0
        # Prevent the integral term from exceeding the limits of the PID output
        if self.rudderI != 0.0:
            if pidRightLimit > pidLeftLimit:
                if self.rudderITerm >= pidRightLimit or self.rudderITerm <= pidLeftLimit:
                    self.rudderITerm = max(min(self.rudderITerm, pidRightLimit), pidLeftLimit)
            else:
                if self.rudderITerm >= pidLeftLimit or self.rudderITerm <= pidRightLimit:
                    self.rudderITerm = max(min(self.rudderITerm, pidLeftLimit), pidRightLimit)
        
        inputDerivative = coerceAngleToRange(pidInput - self.previousInput, -180, 180)
        pidOutputValue = courseError * self.rudderP + self.rudderITerm - inputDerivative * self.rudderD / self.controlInterval
        self.previousInput = pidInput
        
        # conversion of that output value (from -100 to 100) to a physical rudder orientation
        if pidRightLimit > pidLeftLimit:
            # If we can't turn in the direction we want to, go neutral
            # Set us between the limits so rudder = 0.
            if (pidLeftLimit > 0 and pidOutputValue < 0) or (pidRightLimit < 0 and pidOutputValue > 0):
                constrainedPidValue = 0
            else:
                constrainedPidValue = max(min(pidOutputValue, pidRightLimit), pidLeftLimit)
            rudderPos = lerp(constrainedPidValue, -100, 100, turnLeftDirection, turnRightDirection)
        else:
            if (pidLeftLimit < 0 and pidOutputValue > 0) or (pidRightLimit > 0 and pidOutputValue < 0):
                constrainedPidValue = 0
            else:
                constrainedPidValue = max(min(pidOutputValue, pidLeftLimit), pidRightLimit)
            rudderPos = lerp(constrainedPidValue, -100, 100, turnRightDirection, turnLeftDirection)
        rudderPos = coerceAngleToRange(rudderPos - rudderRange, -180, 180)
        
        # make sails maximize force. See points of sail for reference.
        absoluteOffWind = coerceAngleToRange(self.trueWindDirection - self.boatHeading, -180, 180)
        mainPos = RelativeDirection(lerp(abs(absoluteOffWind), self.minPointingAngle, self.runningAngle, 0.0, 90.0))
        jibPos = mainPos
        
        # correct sign
        if absoluteOffWind < 0:
            mainPos = -mainPos
            jibPos = -jibPos
        
        # Use the sails to help us steer to our desired course
        # Turn to the wind by pulling the mainsail in and freeing the jib
        # Turn away by pulling the jib in and freeing the mailsail
        offWindAngle = coerceAngleToRange(self.apparentWindDirection - self.boatHeading, -180, 180)
        maxTurnAngle = offWindAngle
        if maxTurnAngle > 0:
            maxTurnAngle -= maxTurnOffset
        else:
            maxTurnAngle += maxTurnOffset
        
        # PI control
        sailPTerm = courseError * self.sailP
        self.sailITerm += courseError * self.sailI * self.controlInterval
        # Prevent value from exceeding the maximum effect value at 100
        self.sailITerm = max(min(self.sailITerm, 100), -100)
        # Prevent value from having the wrong effect if the course changes a lot
        # We don't want inertia in the wrong way.
        if (self.sailITerm > 0 and courseError < -10) or (self.sailITerm < 0 and courseError > 10):
            self.sailITerm = 0.0
        
        turnValue = abs(sailPTerm + self.sailITerm)
        
        # Putting the non zero checks on courseError here help stability
        # We really do not want to oscillate between towards and wawy from the wind.
        if ((courseError > 5.0 and offWindAngle > 0.0) or
            (courseError < -5.0 and offWindAngle < 0.0)):
            # Sail more towards the wind
            mainPos = lerp(turnValue, 0, 100, mainPos, maxTurnOffset)
            jibPos = lerp(turnValue, 0, 100, jibPos, max(min(maxTurnAngle, 90 - maxTurnOffset), -90 + maxTurnOffset))
        else:
            # Sail more away from the wind
            mainPos = lerp(turnValue, 0, 100, mainPos, max(min(maxTurnAngle, 90 - maxTurnOffset), -90 + maxTurnOffset))
            jibPos = lerp(turnValue, 0, 100, jibPos, maxTurnOffset)
        
        # If the mainsail is very closed, then
        # keep jib from being too closed, open at least a minimum number of degrees
        if abs(mainPos) < self.minClosedAngle and abs(jibPos) < self.minClosedAngle:
            jibPos = -self.minClosedAngle if jibPos < 0 else self.minClosedAngle
            
        # Now that calculations are done, since we can't control which way the sails go,
        # we only publish positive values for them
        mainPos = abs(mainPos)
        jibPos = abs(jibPos)

        #Is it a big enough change that we should republish?  This reduces servo jitters
        if (abs(mainPos - self.reqedMain) > self.mainTolerance or
            abs(jibPos - self.reqedJib) > self.jibTolerance or 
            abs(rudderPos - self.reqedRudder) > self.rudderTolerance):
          self.reqedMain = mainPos
          self.reqedJib = jibPos
          self.reqedRudder = rudderPos
          self.publish_positions() #Actually publish the request

    def listener(self):
        rospy.init_node("sailsRudder")
        rospy.Subscriber("/airmar_data", AirmarData, self.airmar_callback())
        rospy.Subscriber("/target_heading", TargetHeading, self.tactics_callback())
        rospy.spin()

if __name__ == "__main__":
    SailsRudder().listener()

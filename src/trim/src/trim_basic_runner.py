#!/usr/bin/env python
#trim_basic_runner.py		Eric Anderson 11/15
#Implements a bare bones sail controller that cues off the airmar
#and linearly maps feasible sailing directions to feasible sail positions.
#Because of how our servo drivers work, we can use an abstract sail trim scale 
#here from 0 (max trim) to 100 (max ease)



#Basic Imports
import roslib
import rospy
import sys
import time

#Custom imports
from compassCalc import *

#Import message types
from airmar.msg import AirmarData   #Read from
from trim.msg import SailPos        #Write to

class Trim():
  def __init__(self):
    self.pub_sailPos = rospy.Publisher("/sail_pos", SailPos, queue_size = 10)  #Set up our publisher
    self.reqedMain = 100.0      #"Requested" positions i.e. those that were last published
    self.reqedJib = 100.0       #Init is fully eased sails (avoids mechanical breakage?)

    #CONSTANTS
    self.pointing_angle = 50.0 #THIS IS THE HARDCODED POINTING ANGLE IN TACTICS!  IT MUST BE ADJUSTED IF THE VALUE IN TACTICS IS CHANGED!!!
    self.running_angle = 165.0 #THIS IS  " " " " RUNNING ANGLE " " " " " "

    self.mainTolerance = 5.0  #The difference at which point we republish
    self.jibTolerance = 5.0

  def publish_trim(self):
    sailPos = SailPos()
    sailPos.mainPos = self.reqedMain
    sailPos.jibPos = self.reqedJib
    self.pub_sailPos.publish(sailPos)

  def airmar_callback(self, data):
    #TODO: is airmar apWndDir relative to the boat or absolute?
    #If Absolute:
    windAngle = compass_diff(data.apWndDir,data.heading)
    #If relative
    windAngle = abs(data.apWndDir)

    #mainPos and jibPos are where we would put the sails based on *this* calculation (we won't necessarily do it)
    if(windAngle < self.pointing_angle):      
      mainPos = 0
      jibPos = 0
    elif(windAngle > self.running_angle):    
      mainPos = 100.0
      jibPos = 100.0
    else:                       #Actual linear mapping
      mainPos = 100 * (windAngle - self.pointing_angle) / (self.running_angle - self.pointing_angle)
      jibPos = mainPos

    #Is it a big enough change that we should republish?  This reduces servo jitters
    if abs(mainPos - self.reqedMain) > 5 or abs(jibPos - self.reqedJib) > 5:
      self.reqedMain = mainPos
      self.reqedJib = jibPos
      self.publish_trim()        #Actually publish the request

  def listener(self):
    rospy.init_node("trim")
    rospy.Subscriber("/airmar_data", AirmarData, self.airmar_callback())
    rospy.spin()   #All we do is listen to the airmar

if __name__ == "__main__":
  Trim().listener()

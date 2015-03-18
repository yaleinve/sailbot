#!/usr/bin/env python
import time
import roslib
import rospy
import sys

from captain.msg import LegInfo
from airmar.msg import AirmarData #TODO not yet implemented.
from competition_info.msg import CompetitionInfo #TODO not yet implemented.
from std_msgs.msg import Bool #the bool for manual mode

#====== Default values on initialization ======#
begin_lat = 0.0
begin_long = 0.0
apWndSpd = 0.0
apWndDir = 0.0
truWndSpd = 0.0
truWndDir = 0.0


goal_lat = 0.0
goal_long = 0.0


isManual = False


def publish_captain():
  if !isManual:
    rospy.init_node("captain")
    pub_leg = rospy.Publisher("/legInfo", LegInfo, queue_size = 10)
    
    leg_info = LegInfo()
    
    leg_info.begin_lat = begin_lat
    leg_info.begin_long = begin_long

    #TODO calculate the end gps coordiante we want to take based on wind dir etc.
    #leg_info.end_lat = 
    #leg_info.end_long = 

    #TODO calculate xte_min and xte_max somehow
    #leg_info.xte_min = 
    #leg_info.xte_max = 

    pub_leg.publish(leg_info)

def airmar_callback(data):
  global begin_lat = data.lat
  global begin_long = data.long
  global apWndSpd = data.apWndSpd
  global apWndDir = data.apWndDir
  global truWndSpd = data.truWndSpd
  global truWndDir = data.truWndDir
  
def competition_info_callback(data):
  #need clarification with spec (what is gpsTarg1 and gpsTarg2?)
  global goal_lat = data.gpsTarg1Lat
  global goal_long = data.gpsTarg1Long

def manual_callback(data):
  global isManual = data.data #the boolan that describes if we are in autonomous or in manual mode.


def listener():
  rospy.init_node("captain")
  rospy.Subscriber("airmarData", AirmarData, airmar_callback)
  rospy.Subscriber("competitionInfo", CompetitionInfo, competition_info_callback)
  rospy.Subscriber("manualMode", Bool, manual_callback)
  publish_captain()
  rospy.spin() 

if __name__ == "__main__":
  listener()





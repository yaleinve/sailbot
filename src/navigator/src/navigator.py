#!/usr/bin/env python
import time
import roslib
import rospy
import sys
from math import radians, cos, sin, asin, sqrt, atan2, degrees

from gpsCalc import *              #import all gps functions
from captain.msg import LegInfo
from airmar.msg import AirmarData
from navigator.msg import TargetCourse

begin_lat = 0.0
begin_long = 0.0
end_lat = 0.0
end_long = 0.0

def publish_navigator():
  rospy.init_node("navigator")
  pub_course = rospy.Publisher("/target_course", TargetCourse, queue_size = 10)
    
  target_course = TargetCourse()

  target_course.course = gpsBearing(begin_lat, begin_long, end_lat, end_long)
  target_course.range = gpsDistance(begin_lat, begin_long, end_lat, end_long)

  pub_course.publish(target_course)


def airmar_callback(data):
  global begin_lat = data.lat
  global begin_long = data.long
  publish_navigator()

def leg_info_callback(data):
  global end_lat = data.end_lat
  global end_long = data.end_long

def listener():
  rospy.init_node("navigator")
  rospy.Subscriber("airmarData", AirmarData, airmar_callback)
  rospy.Subscriber("leg_info", LegInfo, leg_info_callback)
  
  rospy.spin()


if __name__ == "__main__":
  listener()
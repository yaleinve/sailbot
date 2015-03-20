#!/usr/bin/env python
import time
import roslib
import rospy
import sys
from math import radians, cos, sin, asin, sqrt, atan2, degrees


from gpsCalc import *
from navigator.msg import TargetCourse
from captain.msg import LegInfo
from airmar.msg import AirmarData 
from speed_calculator.msg import SpeedStats


#airmar sensor variables to be used in publish_speed_stats()
sog = 0.0
cog = 0.0
twind_dir = 0.0
lat = 0.0
lon = 0.0

#target_course variables to be used in calculating XTE
target_course = 0.0

leg_course = 0.0


#returns the scalar projection of the vector (magnitude direction) onto course.
def vector_projection(magnitude, direction, course):
  return(cos(direction - course) * magnitude)

def publish_speed_stats():
  rospy.init_node("speed_calculator")
  pub_stats = rospy.Publisher("/speed_stats", SpeedStats, queue_size = 10)
    
  speed_stats = SpeedStats()

  speed_stats.VMG = vector_projection(sog, cog, target_course)
  speed_stats.VMGup = vector_projection(sog, cog, twind_dir)
  speed_stats.XTE = vector_projection(target_range, target_course, leg_course)

  pub_stats.publish(speed_stats)

def leg_info_callback(data):
  global leg_course = data.leg_course

def target_course_callback(data):
  global target_course = data.course
  global target_range = data.range

#assuming for now that cog and sog are in knots.
def airmar_callback(data):
  global cog = data.cog
  global sog = data.sog
  global lat = data.lat
  global lon = data.long
  global twind_dir = data.truWndDir
  publish_speed_stats()

def listener():
  rospy.init_node("speed_calculator")

  rospy.Subscriber("navigator", TargetCourse, target_course_callback)
  rospy.Subscriber("leg_info", LegInfo, leg_info_callback)
  rospy.Subscriber("airmar_data", AirmarData, airmar_callback)
    
  rospy.spin()

if __name__ == "__main__":
  listener()

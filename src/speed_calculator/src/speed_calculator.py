#!/usr/bin/env python
import time
import roslib
import rospy
import sys
from math import radians, cos, sin, asin, sqrt, atan2, degrees

from navigator.msg import TargetCourse
from captain.msg import LegInfo
from airmar.msg import AirmarData #TODO not yet implemented.
from speed_calculator.msg import SpeedStats


#airmar sensor variables to be used in publish_speed_stats()
sog = 0.0
cog = 0.0
twind_dir = 0.0
lat = 0.0
lon = 0.0

#target_course variables to be used in calculating XTE(TODO)
target_range = 0.0
target_course = 0.0

#returns the vmg torward the target in the units that were passed into it.
#course is the bearing direction of the vector that we are trying to find the
#component of our velocity to. (can either be the course on the rhumbline or the direction
#of the true wind.)
def calc_vmg(sog, cog, course):
  return(cos(cog - course) * sog)


#======TODO============#
#not really sure how you would be able to find the correct gps point on the target course
#vector. Mathematically simple with points, but a bit confusing with gps points.
def calc_xte(lat, lon, trg_range, trg_course):
  return 0;
#======================#


def publish_speed_stats():
  rospy.init_node("speed_calculator")
  pub_stats = rospy.Publisher("/speed_stats", SpeedStats, queue_size = 10)
    
  speed_stats = SpeedStats()

  speed_stats.VMG = calc_vmg(sog, cog, target_course)
  speeD_stats.VMGup = calc_vmg(sog, cog, twind_dir)
  speed_stats.XTE = calc_xte(lat, lon, target_range, target_course)

  pub_stats.publish(speed_stats)

def target_course_callback(data):
  global target_course = data.course
  global target_range = data.range
  publish_speed_stats()

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
  rospy.Subscriber("airmarData", AirmarData, airmar_callback)
    
  rospy.spin()

if __name__ == "__main__":
  listener()

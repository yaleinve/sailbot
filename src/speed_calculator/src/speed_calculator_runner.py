#!/usr/bin/env python
import time
import roslib
import rospy
import sys
from math import radians, cos, sin, asin, sqrt, atan2, degrees, sqrt


from gpsCalc import *
from navigator.msg import TargetCourse
from captain.msg import LegInfo
from airmar.msg import AirmarData 
from speed_calculator.msg import SpeedStats
from sensor_msgs.msg import NavSatFix
#TODO include statement for gps node


#GLOBALS
cog = 0.0
sog = 0.0
lat = 0.0
lon = 0.0
pub_stats = rospy.Publisher("/speed_stats", SpeedStats, queue_size = 10)



#from leg_info to calculate xte
leg_start_lat = 0.0
leg_start_long = 0.0
 
#airmar sensor variables to be used in publish_speed_stats()
apWndSpd = 0.0
apWndDir = 0.0
twind_dir = 0.0
twind_spd = 0.0
#target_course variables to be used in calculating XTE
target_range = 0.0
target_course = 0.0

leg_course = 0.0


#returns the scalar projection of the vector (magnitude direction) onto course.
def vector_projection(magnitude, direction, course):
  direction, course = list(map(radians, [direction, course]))
  return(cos(direction - course) * magnitude)

#returns a (magnitude,angle) representation of the vector sum
def vector_add(mag1,angle1,mag2,angle2):
  angle1, angle2 = list(map(radians,[angle1, angle2]))
  x = mag1*cos(angle1) + mag2*cos(angle2) 
  y = mag1*sin(angle1) + mag2*sin(angle2)
  retMag = sqrt(x*x + y*y)
  retAngle = (degrees(atan2(y,x))) % 360.0
  return (retMag,retAngle)


#calculate XTE from a target course and range given an intial course we wanted to sail from.
#this will be negative when you are to the left of the leg_course vector ie(you t_course is 
#between 0 and 180 degrees and visa versa)
def calculate_xte(t_range, t_course, l_course):
  angle = (-1 * (t_course - l_course)) if (t_course <= 180) else ((360 - t_course) - l_course)
  xteAbs = sin(radians(angle)) * t_range
  return xteAbs
  
#This is called every time we get new gps and contains all the calculations 
#for the node except for cog and sog, which are done in gps_data_callback()
def publish_speed_stats():
  rospy.loginfo('[speed_calculator] entered pub, data:')
  rospy.loginfo('[speed_calculator]   cog: ' + str(cog) + ', sog: ' + str(sog))
  speed_stats = SpeedStats()

  #Calculations
  speed_stats.cog = cog
  speed_stats.sog = sog

  speed_stats.truWndDir = 0.0 #twind_dir   #Can hardcode a value instead of using a fan!
  speed_stats.truWndSpd = 2.0 #twind_spd   #Same as above
   
  speed_stats.VMG = vector_projection(sog, cog, leg_course)
  #Wind vector is in opposite direction of positive VMGup!!
  speed_stats.VMGup = -1*vector_projection(sog, cog, twind_spd)

  speed_stats.XTE = calculate_xte(target_range, target_course, leg_course)

  pub_stats.publish(speed_stats)

def leg_info_callback(data):
  global leg_course
  global leg_start_lat
  global leg_start_long
  rospy.loginfo('[speed_calculator] im in leg_info_callback')
  leg_course = data.leg_course
  leg_start_lat = data.begin_lat
  leg_start_long = data.begin_long

def target_course_callback(data):
  global target_course
  global target_range

  rospy.loginfo('[speed_calculator] im in target_course_callback')
  target_course = data.course
  target_range = data.range

def airmar_callback(data):
  global twind_dir
  global twind_spd
  global cog
  global sog

  twind_dir = data.truWndDir
  twind_spd = data.truWndSpd
  cog = data.cog  
  sog = data.sog


#This node publishes every time it gets new gps information
def gps_data_callback(data):
  global lat
  global lon

  lat = data.latitude                               #Update position
  lon = data.longitude
 
  publish_speed_stats()

def listener():
  rospy.init_node("speed_calculator")
  rospy.Subscriber("/target_course", TargetCourse, target_course_callback)
  rospy.Subscriber("/leg_info", LegInfo, leg_info_callback)
  rospy.Subscriber("/airmar_data", AirmarData, airmar_callback)
  rospy.Subscriber("fix", NavSatFix, gps_data_callback)    
  rospy.spin()
if __name__ == "__main__":
  listener()

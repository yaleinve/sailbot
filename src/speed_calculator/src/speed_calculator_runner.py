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
#TODO include statement for gps node


#GLOBALS
cog = 0.0
sog = 0.0
lat = 0.0
long = 0.0
 
#airmar sensor variables to be used in publish_speed_stats()
apWndSpd = 0.0
apWndDir = 0.0
tm = rospy.get_time()
#target_course variables to be used in calculating XTE
target_course = 0.0

leg_course = 0.0


#returns the scalar projection of the vector (magnitude direction) onto course.
def vector_projection(magnitude, direction, course):
  return(cos(direction - course) * magnitude)

#returns a (magnitude,angle) representation of the vector sum
def vector_add(mag1,angle1,mag2,angle2):
  angle1, angle2 = list(map(radians,[]))
  x = mag1*cos(angle1) + mag2*cos(angle) 
  y = mag1*sin(angle1) + mag2*sin(angle2)
  retMag = sqrt(x*x + y*y)
  retAngle = (degrees(atan2(y,x))) % 360.0
  return (retMag,retAngle)

#This is called every time we get new gps and contains all the calculations 
#for the node except for cog and sog, which are done in gps_data_callback()
def publish_speed_stats():
  rospy.init_node("speed_calculator")
  pub_stats = rospy.Publisher("/speed_stats", SpeedStats, queue_size = 10)
    
  speed_stats = SpeedStats()

  #Calculations
  speed_stats.cog = cog
  speed_stats.sog = sog

  # tru + (-v) = app
  # so: app + v = tru
  (a,b) = vector_add(apWndSpd,apWndDir, sog, cog)
  speed_stats.truWndDir = b
  speed_stat.truWndSpd = a
   
  speed_stats.VMG = vector_projection(sog, cog, target_course)
  speed_stats.VMGup = vector_projection(sog, cog, truWndDir)

  #Is this correct???
  speed_stats.XTE = vector_projection(target_range, target_course, leg_course)

  pub_stats.publish(speed_stats)

def leg_info_callback(data):
  global leg_course
  leg_course = data.leg_course

def target_course_callback(data):
  global target_course
  global target_range
  target_course = data.course
  target_range = data.range

def airmar_callback(data):
  global apWndDir
  global apWndSpd
#  global twind_dir
  apWndSpd = data.apWndSpd
  apWndDir = data.apWndDir
#  cog = data.cog  #This info is no longer being collected from Airmar...
#  sog = data.sog
#  lat = data.lat
#  lon = data.long
#  twind_dir = data.truWndDir

#This node publishes every time it gets new gps information
def gps_data_callback(data):
  global cog
  global sog
  global lat
  global long
  newTime = rospy.get_time()
  newLat = data.lat
  newLong = data.long
  
  cog = gpsBearing(lat,long,newLat,newLong)
  dx = gpsDistance(lat,long,newLat,newLong)  #Need dt to get sog
  dt = newTime-tm                            #dt in seconds
  sog = dx/dt                                #sog in m/s

  lat = newLat                               #Update position
  long = newLong
  tm = newTime
 
  publish_speed_stats()

def listener():
  rospy.init_node("speed_calculator")

  rospy.Subscriber("navigator", TargetCourse, target_course_callback)
  rospy.Subscriber("leg_info", LegInfo, leg_info_callback)
  rospy.Subscriber("airmar_data", AirmarData, airmar_callback)
  #TODO: add subscriber for gps    
  rospy.spin()

if __name__ == "__main__":
  listener()

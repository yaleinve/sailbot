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
tm = 0.0
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

def calculate_xte(t_range, t_course, l_course):
  xteAbs = abs(sin(t_course - l_course) * t_range)
  bearing = gpsBearing(lat, lon, leg_start_lat, leg_start_long)
  diff = (bearing - leg_course)%360
  if(diff > 180):
    xteAbs *= -1

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

  # tru + (-v) = app
  # so: app + v = tru
  (a,b) = vector_add(apWndSpd,apWndDir, sog, cog)
  speed_stats.truWndDir = b
  speed_stats.truWndSpd = a
   
  speed_stats.VMG = vector_projection(sog, cog, leg_course)
  #Wind vector is in opposite direction of positive VMGup!!
  speed_stats.VMGup = -1*vector_projection(sog, cog, speed_stats.truWndDir)

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
  global apWndDir
  global apWndSpd
#  global twind_dir
  apWndSpd = data.apWndSpd
  apWndDir = data.apWndDir
#  cog = data.cog  #This info is no loner being collected from Airmar...
#  sog = data.sog
#  lat = data.lat
#  lon = data.lon
#  twind_dir = data.truWndDir

#This node publishes every time it gets new gps information
def gps_data_callback(data):
  global cog
  global sog
  global lat
  global lon
  newTime = rospy.get_time()
  newLat = data.latitude
  newLong = data.longitude
  rospy.loginfo('[speed_calculator] im in gps_data_callback')
  
  cog = gpsBearing(lat,lon,newLat,newLong)
  dx = gpsDistance(lat,lon,newLat,newLong)
  
  #rospy.loginfo('[speed_calculator] cog = ' + str(cog) + ' dx = ' + str(dx))

  dt = newTime-tm                            #dt in seconds
  #dt = 2.0 #DEBUGGING 

  sog = dx/dt  if abs(dt > 1*10**-8) else sog #sog in m/s

  lat = newLat                               #Update position
  lon = newLong
  tm = newTime
 
  publish_speed_stats()

def listener():
  rospy.init_node("speed_calculator")
  tm = rospy.get_time()
  rospy.Subscriber("/target_course", TargetCourse, target_course_callback)
  rospy.Subscriber("/leg_info", LegInfo, leg_info_callback)
  rospy.Subscriber("/airmar_data", AirmarData, airmar_callback)
  rospy.Subscriber("fix", NavSatFix, gps_data_callback)    
  rospy.spin()

if __name__ == "__main__":
  listener()

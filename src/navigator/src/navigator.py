#!/usr/bin/env python
import time
import roslib
import rospy
import sys
from math import radians, cos, sin, asin, sqrt, atan2, degrees

from captain.msg import LegInfo
from airmar.msg import AirmarData
from navigator.msg import TargetCourse

begin_lat = 0.0
begin_long = 0.0
end_lat = 0.0
end_long = 0.0

#returns the distance in m between two gps coordinates.
def gpsDistance(lat1, lon1, lat2, lon2):    
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6378100 # Radius of earth in meters.
    return c * r

def gpsBearing(lat1, lon1, lat2, lon2):
  dlon = lon2 - lon1
  y = sin(dlon) * cos(lat2)
  x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
  d = degrees(atan2(y, x))
  #ensure that d is between 0 and 360
  while(not ((d >= 0) and (d <= 360))):
    d += 360 * (d < 0) 
    
  return (d)
  

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
#!/usr/bin/env python
import time
import roslib
import rospy
import sys

from gpsCalc import *              #import all gps functions
from captain.msg import LegInfo
from airmar.msg import AirmarData
from navigator.msg import TargetCourse

begin_lat = 0.0
begin_long = 0.0
end_lat = 0.0
end_long = 0.0
pub_course = rospy.Publisher("/target_course", TargetCourse, queue_size = 10)


def publish_navigator():
  target_course = TargetCourse()

  target_course.course = gpsBearing(begin_lat, begin_long, end_lat, end_long)
  target_course.range = gpsDistance(begin_lat, begin_long, end_lat, end_long)

  pub_course.publish(target_course)


def gps_info_callback(data):
  global begin_lat
  global begin_long
  begin_lat = data.latitude
  begin_long = data.longitude
  publish_navigator()

def leg_info_callback(data):
  global end_lat
  global end_long
  end_lat = data.end_lat
  end_long = data.end_long
  #publish_navigator()     #Likely that gps will update fast enough that this doesn't matter...

def listener():
  rospy.init_node("navigator")
  rospy.Subscriber("/leg_info", LegInfo, leg_info_callback)
  rospy.spin()


if __name__ == "__main__":
  listener()

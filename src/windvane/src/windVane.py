#!/usr/bin/env python
import Adafruit_BBIO.ADC as ADC
import time
import roslib
import rospy
import sys
from windvane.msg import Heading

ADC.setup()

def windvane():
  rospy.init_node("windvane")
  pub_heading = rospy.Publisher("/heading", Heading, queue_size = 10)
  
  heading = Heading()
  
  rate = rospy.Rate(60)

  while not rospy.is_shutdown():
    value = ADC.read("AIN0")
 
    epsilon = .006

    orientation_val = 0
    orientation_string = ""


    if(abs(value - 0.077) < epsilon):
      orientation_val = abs(value - 0.077)
      orientation_string = "West : ~ 270 degrees bearing"
    elif(abs(value - 0.131) < epsilon):
      orientation_val = abs(value - 0.131)
      orientation_string = "North West : ~ 315 degrees bearing"
    elif(abs(value - 0.189) < epsilon):
      orientation_val = abs(value - 0.189)
      orientation_string = "West North West : ~ 292.5 degrees bearing"
    elif(abs(value - 0.231) < epsilon):
      orientation_val = abs(value - 0.231)
      orientation_string = "North : ~ 0 degrees bearing"
    elif(abs(value - 0.311) < epsilon):
      orientation_val = abs(value - 0.311)
      orientation_string = "North North West : ~ 337.5 degrees bearing"
    elif(abs(value - 0.381) < epsilon):
      orientation_val = abs(value - 0.381)
      orientation_string = "South  West : ~ 225 degrees bearing"
    elif(abs(value - 0.411) < epsilon):
      orientation_val = abs(value - 0.411)
      orientation_string = "West South West : ~ 247.5 degrees bearing"
    elif(abs(value - 0.546) < epsilon):
      orientation_val = abs(value - 0.546)
      orientation_string = "North East : ~ 45 degrees bearing"
    elif(abs(value - 0.601) < epsilon):
      orientation_val = abs(value - 0.601)
      orientation_string = "North North East : ~ 22.5 degrees bearing"
    elif(abs(value - 0.716) < epsilon):
      orientation_val = abs(value - 0.716)
      orientation_string = "South : ~ 180 degrees bearing"
    elif(abs(value - 0.758) < epsilon):
      orientation_val = abs(value - 0.758)
      orientation_string = "South South West : ~ 202.5 degrees bearing"
    elif(abs(value - 0.818) < epsilon):
      orientation_val = abs(value - 0.818)
      orientation_string = "South East : ~ 145 degrees bearing"
    elif(abs(value - 0.875) < epsilon):
      orientation_val = abs(value - 0.875)
      orientation_string = "South South East : ~ 167.5 degrees bearing"
    elif(abs(value - 0.908) < epsilon):
      orientation_val = abs(value - 0.908)
      orientation_string = "East : ~ 90 degrees bearing"
    elif(abs(value - 0.917) < epsilon):
      orientation_val = abs(value - 0.917)
      orientation_string = "East North East : ~ 67.5  degrees bearing"
    elif(abs(value - 0.935) < epsilon):
      orientation_val = abs(value - 0.935)
      orientation_string = "East South East : ~ 112.5 degrees bearing"

    heading.orientation_val = orientation_val
    heading.orientation_string = orientation_string
    pub_heading.publish(heading)
    rate.sleep()

if __name__ == "__main__":
  rospy.loginfo("initialize heading node.")
  try:
    windvane()
  except rospy.ROSInterruptException:
    pass






 

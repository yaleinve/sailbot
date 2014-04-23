#!/usr/bin/env python
import Adafruit_BBIO.PWM as PWM
import roslib
import rospy
import sys
from std_msgs.msg import Float64

servo_pin = "P8_13"
duty_min = 3
duty_max = 14.5
duty_span = duty_max - duty_min

PWM.start(servo_pin, (100 - duty_min), 60.0, 1) #args = pin, dutyCycle, frequency, pwmPolarity


def rotate(angle):
  target = angle.data
  if target < 15.00:
    target = 15.00         #Servo jitters if target is out of physical range.  Must correct
  elif target > 178.00:
    target = 178.0

  #print(target)
  duty = 100 - ((target / 180) * duty_span + duty_min)
  PWM.set_duty_cycle(servo_pin, duty)
    

def listener():
  rospy.init_node("servo_control")
  rospy.Subscriber("servo", Float64, rotate)
  rospy.spin() 

if __name__ == "__main__":
  rospy.loginfo("initialize servo node")
  listener()

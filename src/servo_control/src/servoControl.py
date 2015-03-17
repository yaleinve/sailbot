#!/usr/bin/env python
import Adafruit_BBIO.PWM as PWM
import roslib
import rospy
import sys
from radio_control_pongo.msg import PongoServoPos

winch_servo_pin = "P8_13"
rudder_servo_pin = "P8_07"

duty_min = 3
duty_max = 14.5
duty_span = duty_max - duty_min

PWM.start(winch_servo_pin, (100 - duty_min), 60.0, 1) #args = pin, dutyCycle, frequency, pwmPolarity
PWM.start(rudder_servo_pin, (100 - duty_min), 60.0, 1)

def rotate(angle):
  winch_target = angle.winch_pos
  rudder_target = angle.rudder_pos

  if winch_target < 15.00:
    winch_target = 15.00         #Servo jitters if target is out of physical range.  Must correct
  elif winch_target > 178.00:
    winch_target = 178.0
dafruit_BBIO.PWM as PWM
  winch_duty = 100 - ((winch_target / 180) * duty_span + duty_min)
  PWM.set_duty_cycle(winch_servo_pin, winch_duty)

  if rudder_target < 15.00:
    rudder_target = 15.00         #Servo jitters if target is out of physical range.  Must correct
  elif rudder_target > 178.00:
    rudder_target = 178.0
    
  rudder_duty = 100 - ((rudder_target / 180) * duty_span + duty_min)
  PWM.set_duty_cycle(rudder_servo_pin, rudder_duty)
    

def listener():
  rospy.init_node("servo_control")
  rospy.Subscriber("radio", PongoServoPos, rotate)
  rospy.spin() 

if __name__ == "__main__":
  rospy.loginfo("initialize servo node")
  listener()

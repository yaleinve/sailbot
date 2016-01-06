#!/usr/bin/env python
#Eric Anderson						12/15
#Updates servo_control to reflect new output style for edison
#using mraa libraryi

#IMPORTANT: this code assumes that the winch servos cannot turn past all the way in;
#that is, to calibrate, simply remove the drum, rotate the drum to all the way trimmed position,
#turn the servo to angle 0, and push the drum onto the servo
#Because we may actually need the servo to be running at duty_max for this to be the case (if
#the servo is flipped), we provide a bool to set this at the beginning of this code.

import mraa
import roslib
import rospy

import sys
from servoControl.msg import ServoPos 


#Ratchet Specifications: This is needed to map sail angles to sheet lengths
drum_diameter = 1.25  #The internal diameter of the winch drums, in inches             #TODO: measure this
boom_height = 2.00  #The vertical distance from the exit point of the main sheet to the boom #TODO: measure this
boom_length = 12.00  #The length from the gooseneck to the mainsheet attachment point along the boom, in inches.  #TODO: measure this
club_height = 1.00 #The vertical distance from the exit of the jibsheet to the club, in inches  #TODO: measure this
club_length = 12.00 #The length from the club pivot to the jibsheet attachment point, in inches.  #TODO: measure this
main_0_is_duty_max = False  #Which way to orient the servo, see file comment.  #TODO: Set these values appropriately
jib_0_is_duty_max = False


#Given Acshi's configuration, use these pins to write to servos
jib_winch_servo_pin = 6 
main_winch_servo_pin = 5
rudder_servo_pin = 3

#Winch servo specs
winch_period = 20000  # in mircroseconds, corresponds to 50Hz
winch_duty_min = 1100/winch_period
winch_duty_max = 1900/winch_period
winch_degrees = 1260   #Degrees of movement available to servo using these specs


#Rudder servo specs
rudder_period = 20000 #TODO I'm not 100% sure on this stat but it's a safe bet (most servos work at 50 Hz)
rudder_duty_min = 1000/rudder_period
rudder_duty_max = 2000/rudder_period
rudder_degrees = 100  #Degrees of movement available to servo using these specs

#Init the PWM pins
main_pwm = mraa.Pwm(main_winch_servo_pin)
jib_pwm = mraa.Pwm(jib_winch_servo_pin)
rudder_pwm = mraa.Pwm(rudder_servo_pin);

main_pwm.period_us(winch_period)
jib_pwm.period_us(winch_period)
rudder.period_us(rudder_period)

#Enable, but servos won't move until sent a message I think (b/c nothing is written?)
main_pwm.enable(True)
jib_pwm.enable(True)
rudder_pwm.enable(True)			
 
def rotate(angle):
  #The input to this controller is three angles. For sails, 0 corresponds to all the way in
  #For Rudder, 0 Corresponds to centerline
  main_raw = angle.main_angle   #TODO: this message format is not yet specified
  jib_raw = angle.jib_angle
  rudder_raw = angle.rudder_angle

  #Bound the input to our acceptable sail angle range
  if main_raw < 0.00:
    main_raw = 0.00        
  elif main_raw > 90.00:
    main_raw = 90.0

  if jib_raw < 0.00:
    jib_raw = 0.00        
  elif jib_raw > 90.00:
    jib_raw = 90.0

  #Inputs for rudder should be -50 (hard to port) to 50 (hard to starboard)
  if rudder_raw < -50.00:
    rudder_raw = -50.00     
  elif rudder_raw > 50.00:
    rudder_raw = 50.0

  #Map angles to length positions.  For sails, length 0 corresponds to all the way in (0 degrees sail)
  #s is the length of line to ease
  horizontal_s_main = 2.0*math.sin(main_raw/2.0)*boom_length   #Looking down from above the sheet, boom, and centerline form an isoceles triangle
  tot_s_main = math.sqrt(boom_height*boom_height + horizontal_s_main*horizontal_s_main)
  drum_angle_main = tot_s_main/math.pi/drum_diameter/2.0       #What position we need the drum to be at.  Div by 2 for mechanical advantage
  
  #Duty is set with max trim being duty max or duty min, depending on above booleans
  if main_0_is_duty_max:
     main_duty = winch_duty_max - (drum_angle_main/winch_degrees)*(winch_duty_max - winch_duty_min)
  else:
    main_duty = winch_duty_min + (drum_angle_main/winch_degrees)*(winch_duty_max - winch_duty_min)

  #Same for jib
  horizontal_s_jib = 2.0*math.sin(jib_raw/2.0)*club_length     #Looking down from above the sheet, boom, and centerline form an isoceles triangle
  tot_s_jib = math.sqrt(club_height*club_height + horizontal_s_jib*horizontal_s_jib)
  drum_angle_jib = tot_s_jib/math.pi/drum_diameter/2.0	       #What position we need the drum to be at
  
  #Duty is set with max trim being duty max or duty min, depending on above booleans
  if jib_0_is_duty_max:
    jib_duty = winch_duty_max - (drum_angle_jib/winch_degrees)*(winch_duty_max - winch_duty_min)
  else:
    jib_duty = winch_duty_min + (drum_angle_jib/winch_degrees)*(winch_duty_max - winch_duty_min)


  #Rudder is simpler, just look at delta from center
  rudder_duty = rudder_duty_min + 0.5(rudder_duty_max-rudder_duty_min) + rudder_raw/rudder_degrees * (rudder_duty_max-rudder_duty_min)

  #Send pwm signals
  main_pwm.write(main_duty)
  jib_pwm.write(jib_duty)
  rudder_pwm.write(rudder_duty)

def listener():
  rospy.init_node("servo_control")
  rospy.Subscriber("/servo_pos", ServoPos, rotate)
  rospy.spin() 

if __name__ == "__main__":
  rospy.loginfo("initialize servo node")
  listener()

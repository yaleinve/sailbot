#!/usr/bin/env python
#Eric Anderson						12/15
#Updates servo_control to reflect new output style for edison
#using mraa library

#IMPORTANT: this code assumes that the winch servos cannot turn past all the way in;
#that is, to calibrate, simply remove the drum, rotate the drum to all the way trimmed position,
#turn the servo to angle 0, and push the drum onto the servo
#Because we may actually need the servo to be running at duty_max for this to be the case (if
#the servo is flipped), we provide a bool to set this at the beginning of this code.

import mraa
import roslib
import rospy
import math
import sys
from sails_rudder.msg import SailsRudderPos
import numpy as np

#CONFIG CONSTANTS
debug = True         #This hijacks the relays so the board is always in the mode we want
debug_auto =  False   #If we hijack, to we hijack into autonomous or RC mode?

#Ratchet Specifications: This is needed to map sail angles to sheet lengths
#These measurements were taken with 5 holes showing in front of the mast step, shrouds in front hole of chainplate
#IMPORTANT: DRUM DIAMETERS ARE SLIGHTLY DIFFERENT TO RESULT IN CORRECT ANGLES
#INTERPRET THIS AS EITHER DIFFERENT AMOUNTS OF LINE ON EACH WINCH CHANGING
#THE RADIUS ORRRRR AS AN EMPIRICAL CONSTANT TO GET THE MAPPING RIGHT
jib_drum_diameter = 1.6 #The internal diameter of the winch drums, in inches
main_drum_diameter = 1.45 #Same as above.
boom_height = 3.5  #The vertical distance from the exit point of the main sheet to the boom
boom_length = 19.25  #The length from the gooseneck to the mainsheet attachment point along the boom, in inches
boom_rplus = 0.0    #The horizontal length from the mainsheet attachment to the through hull.  If the through hull is further aft of than the mainsheet attachment, this value is POSITIVE
club_height = 2.2   #The vertical distance from the exit of the jibsheet to the club, in inches
club_length = 13.50 #The length from the club pivot to the jibsheet attachment point, in inches.
club_rplus = 0.0    #Same as boom_rplus for jib club


#This is installation dependent, but we are hopefully NOT reinstalling the servos anytime soon
main_0_is_duty_max = True  #Which way to orient the servo, see file comment.
jib_0_is_duty_max = False


#Given Acshi's configuration, use these pins to write to servos
jib_winch_servo_pin = 3
main_winch_servo_pin = 5
rudder_servo_pin = 6

#Winch servo specs
#For compatability with the RC controller's limits, we are only using 0 to 0.97 of the duty range.  The actual
#duty max of the servo is 1900/period.  However, we're going to set it as duty_min + 0.97*(duty_max - duty_min)
#This amount of play gives us 3 1/3 rotations, equivalent to 1200 degrees.  3.5 rotations (as planned) would have been 1260
winch_period = 20000.0  # in mircroseconds, corresponds to 50Hz
winch_duty_min = 1100.0/winch_period
winch_duty_max = 1900.0/winch_period
winch_duty_max = winch_duty_min + 0.97*(winch_duty_max - winch_duty_min)  #See above comment
winch_degrees = 1200.0  #1260.0   #Degrees of movement available to servo using these specs


#Rudder servo specs
rudder_period = 3030.0  #This servo is weird, frequency of 330 Hz (top end) needed
rudder_duty_min = 1000.0/rudder_period
rudder_duty_max = 2000.0/rudder_period
rudder_degrees = 100.0  #Degrees of movement available to servo using these specs

#Global pwm pins:
main_pwm = None
jib_pwm = None
rudder_pwm = None

#A simple class to fool the rotate() method so we can init everything to zero
class RotateMock():
    def __init__(self):
        self.main_angle = 0.0
        self.jib_angle=0.0
        self.rudder_angle=0.0


def hijack(auto):
  if debug:
    AUTONOMOUS_SELECT_PIN = 2   #2 is for rudder, 4 is main/jib
    auto_select_pin = mraa.Gpio(AUTONOMOUS_SELECT_PIN)
    auto_select_pin.dir(mraa.DIR_OUT)   #Really important that you set the input/output of the pin...
    auto_select_pin.write(1 if debug_auto else 0)
    AUTONOMOUS_SELECT_PIN = 4   #2 is for rudder, 4 is main/jib
    auto_select_pin = mraa.Gpio(AUTONOMOUS_SELECT_PIN)
    auto_select_pin.dir(mraa.DIR_OUT)   #Really important that you set the input/output of the pin...
    auto_select_pin.write(1 if debug_auto else 0)



#Init the PWM pins
def initPWMs():
  #Import Globals
  global main_pwm
  global jib_pwm
  global rudder_pwm
  main_pwm = mraa.Pwm(main_winch_servo_pin)
  jib_pwm = mraa.Pwm(jib_winch_servo_pin)
  rudder_pwm = mraa.Pwm(rudder_servo_pin);
  main_pwm.period_us(int(winch_period))
  jib_pwm.period_us(int(winch_period))
  rudder_pwm.period_us(int(rudder_period))
  main_pwm.enable(True)
  jib_pwm.enable(True)
  rudder_pwm.enable(True)
  rotate(RotateMock())


def rotate(angle):
  #Import Globals
  global main_pwm
  global jib_pwm
  global rudder_pwm
  #The input to this controller is three angles. For sails, 0 corresponds to all the way in
  #For Rudder, 0 Corresponds to centerline

  main_raw = angle.mainPos
  jib_raw = angle.jibPos
  rudder_raw = angle.rudderPos
  #print main_raw
  #print jib_raw
  #print rudder_raw

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
  tot_s_main = math.sqrt( (boom_length*math.sin(np.deg2rad(main_raw)))**2 + boom_height**2 + (boom_rplus + boom_length*(1.0-math.cos(np.deg2rad(main_raw))))**2 )

  #horizontal_s_main = 2.0*math.sin(main_raw/2.0)*boom_length   #Looking down from above the sheet, boom, and centerline form an isoceles triangle
  #tot_s_main = math.sqrt(boom_height*boom_height + horizontal_s_main*horizontal_s_main)
  drum_angle_main = tot_s_main/math.pi/main_drum_diameter/2.0*360.0       #What position we need the drum to be at.  Div by 2 for mechanical advantage

  #Duty is set with max trim being duty max or duty min, depending on above booleans
  if main_0_is_duty_max:
     main_duty = winch_duty_max - (drum_angle_main/winch_degrees)*(winch_duty_max - winch_duty_min)
  else:
    main_duty = winch_duty_min + (drum_angle_main/winch_degrees)*(winch_duty_max - winch_duty_min)


  #Bound it by duty cycle for safety
  main_duty = min(max(main_duty,winch_duty_min),winch_duty_max)


  #Same for jib
  tot_s_jib = math.sqrt( (club_length*math.sin(np.deg2rad(jib_raw)))**2 + club_height**2 + (club_rplus + club_length*(1.0-math.cos(np.deg2rad(jib_raw))))**2 )
  #horizontal_s_jib = 2.0*math.sin(jib_raw/2.0)*club_length     #Looking down from above the sheet, boom, and centerline form an isoceles triangle
  #tot_s_jib = math.sqrt(club_height*club_height + horizontal_s_jib*horizontal_s_jib)
  drum_angle_jib = tot_s_jib/math.pi/jib_drum_diameter/2.0*360.0	       #What position we need the drum to be at

  print tot_s_main

  #Duty is set with max trim being duty max or duty min, depending on above booleans
  if jib_0_is_duty_max:
    jib_duty = winch_duty_max - (drum_angle_jib/winch_degrees)*(winch_duty_max - winch_duty_min)
  else:
    jib_duty = winch_duty_min + (drum_angle_jib/winch_degrees)*(winch_duty_max - winch_duty_min)

  #Bound it by duty cycle for safety
  jib_duty = min(max(jib_duty,winch_duty_min),winch_duty_max)

  #Rudder is simpler, just look at delta from center
  rudder_duty = rudder_duty_min + 0.5*(rudder_duty_max-rudder_duty_min) + rudder_raw/rudder_degrees * (rudder_duty_max-rudder_duty_min)

  #Bound it by duty cycle for safety
  rudder_duty = min(max(rudder_duty,rudder_duty_min),rudder_duty_max)

  #print main_duty
  #print jib_duty
  #print rudder_duty
  #Send pwm signals
  main_pwm.write(main_duty)
  jib_pwm.write(jib_duty)
  rudder_pwm.write(rudder_duty)

def listener():
  rospy.init_node("servo_control")
  rospy.Subscriber("/sails_rudder_pos", SailsRudderPos, rotate)
  rospy.spin()

if __name__ == "__main__":
  rospy.loginfo("initialize servo node")
  initPWMs()
  rospy.loginfo("debug mode set to: " + str(debug))
  #Hijack sevo control?
  if debug:
    hijack(debug_auto)
  listener()

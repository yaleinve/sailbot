#!/usr/bin/env python
#Eric Anderson						12/15
#Updates servo_control to reflect new output style for edison
#using mraa library
import mraa
import roslib
import rospy
import sys
from radio_control_pongo.msg import PongoServoPos


#TODO: update these pins to match edison out
jib_winch_servo_pin = "TODO"
main_winch_servo_pin = "TODO"
rudder_servo_pin = "TODO"

#Winch servo specs
winch_period = 20000  # in mircroseconds, corresponds to 50Hz
winch_duty_min = 1100/winch_period
winch_duty_max = 1900/winch_period
winch_degrees = 1260   #Degrees of movement available to servo using these specs


#Rudder servo specs
#TODO research rudder servo specs and add here
rudder_period = "TODO"
rudder_duty_min = "TODO"
rudder_duty_max = "TODO"

#TODO: are these the units we wish to use?
#Empirical tuning to say what max in/out should actually be, in degrees.
# 0 = max trim in for servo
# 1260 = max trim out for servo

#For now, set to very conservative values around the middle of the servo's range of motion
main_max_trim_deg = 1.75 * 360 - 90   #90 degrees tighter than neutral
main_min_trim_deg = 1.75 * 360 + 90   #90 degrees looser than neutral
jib_max_trim_deg = 1.75 * 360 - 90    #90 degrees tighter than neutral
jib_min_trim_deg = 1.75 * 360 + 90    #90 degrees looser than neutral

#TODO Empirical numbers for rudder?


#Init the PWM pins
main_pwm = mraa.Pwm(main_winch_servo_pin)
jib_pwm = mraa.Pwm(jib_winch_servo_pin)
rudder_pwm = mraa.Pwm(rudder_servo_pin);

main_pwm.period_us(winch_period)
jib_pwm.period_us(winch_period)
rudder.period_us(rudder_period)

main_pwm.enable(True)
jib_pwm.enable(True)
rudder_pwm.enable(True)			
 
#For mechanical safety, always ease upon starting.
#TODO
 
#pwm.write((duty_min + (duty_max-duty_min)*duty)/period);       #Example code for mraa write


def rotate(angle):
  main_raw = angle.main_pos   #TODO: this message format is not yet specified
  jib_raw = angle.jib_pos
  rudder_raw = angle.rudder_pos

  #Inputs for winches should be 0 to 100 (abstracted) with zero being max trim in
  if main_raw < 0.00:
    main_raw = 0.00        
  elif main_raw > 100.00:
    main_raw = 100.0

  if jib_raw < 0.00:
    jib_raw = 0.00        
  elif jib_raw > 100.00:
    jib_raw = 100.0

  #Inputs for rudder should be -50 (hard to port) to 50 (hard to starboard)
  if rudder_raw < -50.00:
    rudder_raw = -50.00     
  elif rudder_raw > 50.00:
    rudder_raw = 50.0
 
  #The input to this program is on a scale of 0 to 100 for each winch servo.  We normalize by that range.  Then, we map that to the
  #manual tuning range we have defined.  Finally, we map that desired angle to the duty cycle given the total range of the servo.
  main_duty = winch_duty_min + (main_raw / 100.0 * (main_max_trim_deg - main_min_trim_deg) / winch_degrees * (winch_duty_max - winch_duty_min))
  jib_duty = winch_duty_min + (jib_raw / 100.0 * (main_max_trim_deg - main_min_trim_deg) / winch_degrees * (winch_duty_max - winch_duty_min))
  #TODO: rudder duty?  
  rudder_duty = rudder_duty_min + ((rudder_raw + 50.0))    #TODO not yet complete

  #Send pwm signals
  main_pwm.write(main_duty)
  jib_pwm.write(jib_duty)
  rudder_pwm.write(rudder_duty)

def listener():
  rospy.init_node("servo_control")
  rospy.Subscriber("radio", PongoServoPos, rotate)  #TODO: Update this to reflect that we are working with Ratchet
  #TODO: subscribe to a manual tuning thing as well?  Or, create a manual tuning node separately?
  rospy.spin() 

if __name__ == "__main__":
  rospy.loginfo("initialize servo node")
  initPWMs()
  listener()

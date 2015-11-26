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
rudder_period = 20000 #TODO I'm not 100% sure on this stat but it's a safe bet (most servos work at 50 Hz)
rudder_duty_min = 1000/rudder_period
rudder_duty_max = 2000/rudder_period
rudder_degrees = 100

#Empirical tuning to say what max in/out should actually be, in degrees.
# 0 = max trim in for servo
# 1260 = max trim out for servo

#For now, set to very conservative values around the middle of the servo's range of motion
main_max_trim_deg = 1.75 * 360 - 90   #90 degrees tighter than neutral
main_min_trim_deg = 1.75 * 360 + 90   #90 degrees looser than neutral
jib_max_trim_deg = 1.75 * 360 - 90    #90 degrees tighter than neutral
jib_min_trim_deg = 1.75 * 360 + 90    #90 degrees looser than neutral
rudder_center_deg = 50.0	      #Servo turns through 100 degrees with above settings
rudder_turn_deg = 30.0		      #For now, allow 30 degrees of throw


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
  #TODO: double check these ranges!!!
  main_duty = winch_duty_min + ((main_min_trim_deg + main_raw/100.0 * (main_max_trim_deg - main_min_trim_deg))/winch_degrees) * (winch_duty_max - winch_duty_min)
  jib_duty = winch_duty_min + ((jib_min_trim_deg + jib_raw/100.0 * (jib_max_trim_deg - jib_min_trim_deg))/winch_degrees) * (winch_duty_max - winch_duty_min)
  rudder_duty = rudder_duty_min + (rudder_raw / 50.0 * rudder_turn_deg + rudder_center_deg)/rudder_degrees * (rudder_duty_max - rudder_duty_min)

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
  listener()

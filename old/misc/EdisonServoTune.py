#!/usr/bin/python

#import mraa
import time

#TODO : actually output the angles to the servos
#TODO: handle the possibility that jib servo might be reverse angles from main servo (0 might be full ease, but we should remap it)

#TODO: update these pins to match edison out
#jib_winch_servo_pin = "TODO"
#main_winch_servo_pin = "TODO"
#rudder_servo_pin = "TODO"

#Winch servo specs
#winch_period = 20000  # in mircroseconds, corresponds to 50Hz
#winch_duty_min = 1100/winch_period
#winch_duty_max = 1900/winch_period
#winch_degrees = 1260   #Degrees of movement available to servo using these specs


#Rudder servo specs
#rudder_period = 20000 #TODO I'm not 100% sure on this stat but it's a safe bet (most servos work at 50 Hz)
#rudder_duty_min = 1000/rudder_period
#rudder_duty_max = 2000/rudder_period
#rudder_degrees = 100

#Init the PWM pins
#main_pwm = mraa.Pwm(main_winch_servo_pin)
#jib_pwm = mraa.Pwm(jib_winch_servo_pin)
#rudder_pwm = mraa.Pwm(rudder_servo_pin);

#main_pwm.period_us(winch_period)
#jib_pwm.period_us(winch_period)
#rudder.period_us(rudder_period)

#Enable, but servos won't move until sent a message I think (b/c nothing is written?)
#main_pwm.enable(True)
#jib_pwm.enable(True)
#rudder_pwm.enable(True)			
 
def main():
  print("\n\n\tRatchet Servo Tuner\n\n")
  print("Enter a command of the form [j|m|r]ANGLE where ANGLE is a number ranging from 0 to 1260 for main/jib and 0 to 100 for Rudder")
  print("Example: j50   (will turn the jib servo to 50 degrees absolute)")
  while True:
    val = raw_input("Command: ")
    c = val[0]
    angle = float(val[1:])
    if c == "r":
      if angle < 0 or angle > 100:
        print("Angle (" + str(angle) + ") is out of range for rudder- must be in [0,100]")
      else:
        print("Moving RUDDER to angle: " + str(angle))
        #TODO
    elif c == "j":
      if angle < 0 or angle > 1260:
        print("Angle (" + str(angle) + ") is out of range for jib- must be in [0,1260]")
      else:
        print("Moving JIB to angle: " + str(angle))
        #TODO
    elif c == "m":
      if angle < 0 or angle > 1260:
        print("Angle (" + str(angle) + ") is out of range for main- must be in [0,1260]")
      else:
        print("Moving MAIN to angle: " + str(angle))
        #TODO
    else:
      print("Illegal leading character: must be one of j,r,m")


if __name__ == "__main__":
    main()

#!/usr/bin/python



#conclusion that 3 is rudder, 5 is main sail, and 6 is jib for mraa pin numbers 

import mraa
import time

def main():

  #Setup
  duty = 0;
  val = 0;  
    


  #Servo Constants
  period = 20000
  duty_min = 1100.0 /period;
  duty_max = 1900.0 /period;

  #MRAA setup

  #switch to autonomous mode.
  AUTONOMOUS_SELECT_PIN = 2
  auto_select_pin = mraa.Gpio(AUTONOMOUS_SELECT_PIN)
  auto_select_pin.dir(mraa.DIR_OUT)
  auto_select_pin.write(1)


  PWM_PIN = 6;    #pwm pins are the same as listed on the breakout board. (3,5,6)
  pwm = mraa.Pwm(PWM_PIN);
  pwm.period_us(period);   #Period in microseconds, corresponds to 50Hz like the winch motors.
  pwm.enable(True);
  

  while True:
    val = input("Enter a duty cycle in [0,1]:  ");
    print(val);
    if val < 0 or val > 1:
      print("Duty is out of range");
      continue;
    duty = val;
    writeval = (duty_min + (duty_max-duty_min)*duty)
    print writeval
    pwm.write(writeval);       #Scale it according to our ranges

 
if __name__ == "__main__":
    main()

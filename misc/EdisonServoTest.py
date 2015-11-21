#!/usr/bin/python

import mraa
import time

def main():

  #Setup
  duty = 0;
  val = 0;  
    

  #Servo Constants
  period = 20000;
  duty_min = 1100/period;
  duty_max = 1900/period;
  

  #MRAA setup
  PWM_PIN = 5;    #TODO: Change this
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
    pwm.write((duty_min + (duty_max-duty_min)*duty)/period);       #Scale it according to our ranges

 


if __name__ == "__main__":
    main()

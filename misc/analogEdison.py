#!/usr/bin/python

#conclusion that 6 is rudder, 5 is main sail, and 3 is jib for mraa pin numbers
#important!! with fst6 controller, switches cause jump from about 37 to 67 (raw values from analog read).  We can just put a separator in between
#to figure out which mode we're in
import mraa
import time

def main():

  #Setup
  duty = 0;
  val = 0;



  #Servo Constants, duty of 1 outputs abou 3.3 volts
  #Rudder
  period = 3030.0
  duty_min = 1000.0 /period;
  duty_max = 2000.0 /period;

  #Winches
  # period = 20000.0
  # duty_min = 1100.0/period
  # duty_max=1900.0 /period

  #main is 90 degrees out at duty cycle 0.3 (1 is all the way in)
  #jib is 90 degrees out at duty cycle 0.7 (0 is all the way in)


  #MRAA setup

  #switch to autonomous mode.
  AUTONOMOUS_SELECT_PIN = 2   #2 is for rudder, 4 is main/jib
  auto_select_pin = mraa.Gpio(AUTONOMOUS_SELECT_PIN)
  auto_select_pin.dir(mraa.DIR_OUT)   #Really important that you set the input/output of the pin...
  auto_select_pin.write(1)

  #PWM_PIN = 6;    #pwm pins are the same as listed on the breakout board. (3 jib, 5 main, 6 rudder)
  #pwm = mraa.Pwm(PWM_PIN);
  #pwm.period_us(int(period));   #Period in microseconds, corresponds to 50Hz like the winch motors.
  #pwm.enable(True);


  #Analog config
  #ANALOG_PIN = 1        #Reading from aux 1
  #ANALOG_PIN=0          #Reading from aux 2
  ANALOG_PIN=5           #Reading from Rudder
  aio = mraa.Aio(ANALOG_PIN)


  while True:
    val = input("Enter a duty cycle in [0,1]:  ")
    print(val);
    if val < 0 or val > 1:
      print("Duty is out of range");
      continue;
    duty = val;
    writeval = (duty_min + (duty_max-duty_min)*duty)
    print writeval
    val = aio.read()
    #pwm.write(writeval);       #Scale it according to our ranges

    print('value from analaog ' + str(ANALOG_PIN) + ": " + str(aval))
    print('equivalent to ' + str(aio.read()/1024.0 * 5.0*2.0) + ' volts')
    #print('Aux disambiguate: ' + str(aio.read() > 50))

if __name__ == "__main__":
    main()

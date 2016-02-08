#!/usr/bin/env python

#pwmAioMap.py
#pulls in servo controller functions
#Writes consecutive angles to servos
#Reads ADC

import sys
import numpy as np
sys.path.append('/home/edison/sailbot/src/servo_control/src')
import servoControl as servo
import mraa
import time

#Simple object similar to ros pub that we can pass to rotate
class hackMsg():
	def __init__(self,main,jib,rudder=0):
		self.main_angle = main
		self.jib_angle = jib
		self.rudder_angle = rudder

def main():
    #TODO: not sure 100 samples is any more consistent than 16
    n_samples = 100      #The number of analog reads we average to get servo position
    avg_start = 25       #The internal range of data to use in the mean (get rid of outliers)
    avg_end = 75        # " " " upper bound

    analog = mraa.Aio(4)	#Read from the rudder ADC, fine as long as as all ADC's created equal
    servo.initPWMs()
    servo.hijack()		#Turn the relays to autonomous mode
    for deg in range(0,90,5):
        print("Deg: " + str(deg))
        servo.rotate(hackMsg(deg,0))	#Output the servo
        time.sleep(1)
        for test in range(5):
            print("Test #: " + str(test))
            samps = []
  			#Average across middle 50% of multiple reads to reduce noise from ADC
            for samp in range(n_samples):
    		    samps.append(analog.read())

  			#Take the middle 50% of samples and average them
            samps.sort()
            val = np.mean(samps[avg_start:avg_end])
            print("Avg ADC value: " + str(val))
            time.sleep(0.1)  #TODO this might be an important sleep, not quite sure
        print("")

if __name__ == "__main__":
	main()

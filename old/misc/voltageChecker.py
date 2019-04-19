#!/usr/bin/env python

#A simple script that checks the battery voltage on a pin and prints out a warning if the voltage drops below nominal

import os
import sys
import mraa
import time
import numpy as np

print("Voltage Checking Script has been invoked")
VOLTAGE_PIN=5
vPin = mraa.Aio(VOLTAGE_PIN)
delay = 60
while True:
    #Check every minute
    voltage = []
    for i in range(100):
        voltage.append(vPin.read()/ 1024.0 * 5.0 * 2.0)
        av = np.array(voltage).mean()
    if voltage < 6.9:                           #Nominal voltage of our battery is 7.4
        warn = "Low Voltage Detected! Voltage = " + str(av)
        os.system("echo " + warn + " | wall")
        delay = 20
    else:
        delay = 60
    time.sleep(delay)

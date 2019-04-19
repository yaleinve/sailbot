#!/usr/bin/env python
# sail_bagger.py              Eric Anderson Feb 2016
#Reads from airmar and ADC's, stores data to csv file

#Import statements
import roslib
import rospy
import sys
import time
import mraa
import csv
import datetime as dt
import numpy as np

#Import the signatures/headers for the message types we have created
from airmar.msg import AirmarData


def initGlobals():
  #File writer
  global writer  

  #Hardware related globals
  global main_aio
  global jib_aio
  global collect_aio


  #Sailing related globals
  global heading
  global apWndDir
  global apWndSpd
  global amrRoll
  global cog
  global sog
  global vmg
  global vmgUp
  global truWndDir

  jib_aio = mraa.Aio(2)
  main_aio = mraa.Aio(3)    #TODO confirm these!!!
  collect_aio = mraa.Aio(0) #Aux 2 for data collection
  amrRoll = 0.0
  heading = 0.0
  apWndDir = 0.0
  apWndSpd = 0.0
  cog = 0.0
  sog = 0.0
  vmg = 0.0
  vmgUp = 0.0
  truWndDir = 0.0

#Put the data from the airmar message into global variables
def airmar_callback(data):
  global collect_aio

  global amrRoll
  global heading
  global apWndSpd
  global apWndDir
  global vmg
  global vmgUp
  global cog
  global sog
  global truWndDir

  amrRoll = data.amrRoll
  heading  = data.heading
  apWndSpd = data.apWndSpd
  apWndDir = data.apWndDir
  vmg = data.VMG
  vmgUp = data.VMGup
  cog = data.cog
  sog = data.sog
  truWndDir = data.truWndDir

  if collect_aio > 50:          #Aux switch turned on
    writeData()

def writeData():
  n_samples = 16      #The number of analog reads we average to get servo position
  avg_start = 4       #The internal range of data to use in the mean (get rid of outliers)
  avg_end = 12        # " " " upper bound

  global main_aio
  global jib_aio

  global amrRoll
  global heading
  global apWndSpd
  global apWndDir
  global vmg
  global vmgUp
  global cog
  global sog
  global truWndDir

  jib_samps = []
  main_samps = []
  #Average across middle 50% of multiple reads to reduce noise from ADC
  for samp in range(n_samples):
    jib_samps.append(jib_aio.read())
    main_samps.append(main_aio.read())

  #Take the middle 50% of samples and average them
  jib_samps.sort()
  main_samps.sort()

  jval = np.mean(jib_samps[avg_start:avg_end])
  mval = np.mean(main_samps[avg_start:avg_end])

  #TODO: process these values to get main/jib positions

  writer.writerow([apWndSpd,apWndDir,mval,jval,sog,cog,heading,amrRoll])  #Write a row to the csv

def listener():
  initGlobals()
  rospy.init_node("sail_bagger")  #Must init node to subscribe

  global writer
  date = dt.datetime.today().strftime("%m_%d_%Y_%H:%M")
  fname = date + '_data.csv'  #Minute specific name to avoid overwriting data
  with open('/home/edison/sailbot/src/sail_bagger/data/' + fname,'w') as f:   #Write to an absolute path so it works regardless of where we launch from
    writer = csv.writer(f)
    rospy.Subscriber("/airmar_data", AirmarData, airmar_callback)
    rospy.spin()


if __name__ == "__main__":
  listener() 	#Listen to our subscriptions

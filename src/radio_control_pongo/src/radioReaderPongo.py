#!/usr/bin/env python
import Adafruit_BBIO.ADC as ADC
import roslib
import rospy
import sys
import time
from radio_control_pongo.msg import PongoServoPos
#Reads Channels 1-3 from the radio controller and publishes to ServoPos topic.

ADC.setup()

def radioRead():
  while not rospy.is_shutdown():
    rospy.init_node("radio_control_pongo")
    pub_radio = rospy.Publisher("/radio", PongoServoPos, queue_size = 10)
    radio = PongoServoPos() #make an instance of the message type
    rate = rospy.Rate(5) #set the refresh rate in HZ

    channel1 = ADC.read("AIN1")
    #print("channel 1: " + str(channel1))
    channel2 = ADC.read("AIN3")
    #print("channel 2: " + str(channel2) + "\n")

    output_start = 0.0 #the lowest angle we will be sending to the servo
    output_end = 180.0 # ''' highest '''

    input_start = .20 
    input_end = .10

    winchOutput  =  (channel1 - input_start) / (input_end - input_start) * (output_end - output_start) + output_start
    rudderOutput =  (channel2 - input_start) / (input_end - input_start) * (output_end - output_start) + output_start
    
    radio.winch_pos = winchOutput
    radio.rudder_pos = rudderOutput
    
    pub_radio.publish(radio)
    rate.sleep()

if __name__ == "__main__":
  rospy.loginfo("initialize radio node")
  try:
    radioRead()
  except rospy.ROSInterruptException:
    pass
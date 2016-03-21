#!/usr/bin/env python

#Updated by Eric Anderson 12/15 to adapt for Edison and recent arch changes,
#namely that we rely solely on airmar (and camera) for sensing - no navsat

import rospy
import random
import serial
import pynmea2
import pdb
import time
import mraa

from math import radians, cos, sin, asin, sqrt, atan2, degrees
from airmar.msg import AirmarData
from captain.msg import LegInfo

#Debugging airmar as steering wheel
#from servo_control.msg import ServoPos

class Airmar:
  def __init__(self, speedCalculatorInstance):
    #Set the publishing rate of the airmar to 2Hz for now
    self.pubInterval = 0.2      # Seconds between publishes

    self.heading = 0
    self.amrRoll = 0
    self.truWndDir = 0
    self.truWndSpd = 0
    self.cog = 0
    self.lat = 0
    self.long = 0
    self.sog = 0
    self.wndUnits = "K"
    self.apWndDir = 0
    self.apWndSpd = 0

    #Store the speed calculator
    self.spd = speedCalculatorInstance

    #Initialize a publisher
    self.pub = rospy.Publisher('/airmar_data', AirmarData, queue_size = 10)
#   Debugging, turns airmar into steering wheel
#    self.servoPub = rospy.Publisher('/servo_pos', ServoPos, queue_size = 10)



    #Use mraa to initialize edison pins 0 and 1 to serial
    mraa.Uart(0)
    #...but, after initialized, let's use the pyserial library
    self.ser = serial.Serial('/dev/ttyMFD1', 4800, timeout=1)
    self.ser.readline()

  def update_data(self, debug=False):
    '''
    Fetches the latest data off the serial wire from the Airmar
    debug -- if True, publish fake data, else, publish real data (Not yet implemented)
    '''
    line = self.ser.readline()
    if "," in str(line):
      try:
        msg = pynmea2.parse(line)
        if msg.sentence_type == 'MWV':
          self.apWndSpd = msg.wind_speed if msg.wind_speed != None else self.apWndSpd
          self.wndUnits = msg.wind_speed_units if msg.wind_speed_units != None else self.wndUnits
          self.apWndDir = msg.wind_angle if msg.wind_angle != None else self.apWndDir
        elif msg.sentence_type == 'MWD':
          self.truWndSpd = msg.wind_speed_knots if msg.wind_speed_knots != None else self.truWndSpd
          self.truWndDir = msg.direction_true if msg.direction_true != None else self.truWndDir
        elif msg.sentence_type == 'HDT': # change to HDG for magnetic heading
          self.heading = msg.heading if msg.heading != None else self.heading
        elif msg.sentence_type == 'VTG':
          self.cog = msg.true_track if msg.true_track != None else self.cog # change to mag_track for magnetic cog
          self.sog = msg.spd_over_grnd_kts if msg.spd_over_grnd_kts != None else self.sog# can also support kmph
        elif msg.sentence_type == 'GGA':
          self.lat = msg.latitude if msg.latitude != None else self.lat
          self.long = msg.longitude if msg.longitude != None else self.long
        elif msg.sentence_type == 'XDR' and msg.type == 'A':
          self.amrRoll = float(str(msg).split(",")[6]) if str(msg).split(",")[6] != None else self.amrRoll
                    # The pynmea2 library can't parse our XDR messages properly
                    # This basically grabs the roll value manually from the message (it's in a standard form)


      except Exception, e:
        rospy.loginfo("[airmar] Error!")
        rospy.loginfo(str(e))



  def airmar_pub(self):
#    DEBUGGING AIRMAR AS STEERING WHEEL FOR SERVO
#    debug_msg = ServoPos()
#    debug_msg.main_angle = 0.0
#    debug_msg.jib_angle = 0.0
#    debug_msg.rudder_angle = self.heading - 50 if self.heading < 100 and self.heading >= 0 else 0.0
#    self.servoPub.publish(debug_msg)

    airmar_data_msg = AirmarData()
    airmar_data_msg.heading = self.heading
    airmar_data_msg.amrRoll = self.amrRoll
    airmar_data_msg.apWndDir = self.apWndDir
    airmar_data_msg.apWndSpd = self.apWndSpd
    airmar_data_msg.cog = self.cog
    airmar_data_msg.sog = self.sog
    airmar_data_msg.truWndSpd = self.truWndSpd
    airmar_data_msg.truWndDir = self.truWndDir
    airmar_data_msg.lat = self.lat
    airmar_data_msg.long = self.long      #FIXME: does 'long' overwrite a python keyword?
    
    self.target_range = gpsDistance(self.lat, self.long, spd.end_lat, spd.end_long)
    self.target_course = gpsBearing(self.lat, self.long, spd.end_lat, spd.end_long)

    #Calculated in speed calculator
    airmar_data_msg.VMG = self.spd.calc_VMG(float(self.sog), float(self.cog))
    airmar_data_msg.VMGup  = self.spd.calc_VMGup(float(self.sog),float(self.cog),float(self.truWndDir))
    airmar_data_msg.XTE = self.spd.calc_xte(self.target_course, target_range)

    self.pub.publish(airmar_data_msg)


class SpeedCalculator:
  def __init__(self):
    #For leg info callback (captain output)
    self.leg_course = 0.0
    self.leg_start_lat = 0.0
    self.leg_start_long = 0.0
    
    self.end_lat = 0.0
    self.end_long = 0.0
    

  def leg_info_callback(self, data):
    self.leg_course = data.leg_course
    self.leg_start_lat = data.begin_lat
    self.leg_start_long = data.begin_long
    self.end_lat = data.end_lat
    self.end_long = data.end_long


  #CALCULATION FUNCTIONS
  #returns the scalar projection of the vector (magnitude direction) onto course.
  def vector_projection(self,magnitude, direction, course):
    direction, course = list(map(radians, [direction, course]))
    return(cos(direction - course) * magnitude)

  #returns a (magnitude,angle) representation of the vector sum
  def vector_add(self,mag1,angle1,mag2,angle2):
    angle1, angle2 = list(map(radians,[angle1, angle2]))
    x = mag1*cos(angle1) + mag2*cos(angle2)
    y = mag1*sin(angle1) + mag2*sin(angle2)
    retMag = sqrt(x*x + y*y)
    retAngle = (degrees(atan2(y,x))) % 360.0
    return (retMag,retAngle)


  #calculate XTE from a target course and range given an intial course we wanted to sail from.
  #this will be negative when you are to the left of the leg_course vector (i.e. your t_course is
  #between 0 and 180 degrees and visa versa)
  def _calculate_xte(self,t_range, t_course, l_course):
    angle = (t_course - l_course) % 360
    angle = (-1 * angle) if (angle <= 180) else (360 - angle)
    xte = sin(radians(angle)) * t_range
    return xte

  def calc_VMG(self, sog, cog):
    return self.vector_projection(sog, cog, self.leg_course)

  def calc_VMGup(self, sog, cog, truWndDir):
    return -1.0*self.vector_projection(sog, cog, truWndDir) #Wind vector is in opposite direction of positive VMGup!!

  def calc_xte(self, target_course, target_range):
    return self._calculate_xte(target_range, target_course, self.leg_course)


if __name__ == '__main__':
    try:
        #Init the ros node
        rospy.init_node('airmar')
        spd = SpeedCalculator()
        rospy.Subscriber("/leg_info", LegInfo, spd.leg_info_callback)
        am = Airmar(spd)
        rospy.loginfo("[airmar] Started airmar node!")

        tm = rospy.get_time()

        while not rospy.is_shutdown():
            am.update_data() # Get the latest data from the airmar
            if (rospy.get_time() - tm > am.pubInterval):
                am.airmar_pub() # Publish the latest data
                tm = rospy.get_time()

    except Exception as e:
        print e

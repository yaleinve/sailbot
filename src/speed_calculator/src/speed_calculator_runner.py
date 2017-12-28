#!/usr/bin/env python
import rospy

from gpsCalc import *
from captain.msg import LegInfo
from airmar.msg import AirmarData
from speed_calculator.msg import SpeedStats


class SpeedCalculator:
    def __init__(self):
        # For airmar callback
        self.lat = 0.0
        self.long = 0.0
        self.truWndDir = 0.0
        self.truWndSpd = 0.0
        self.cog = 0.0
        self.sog = 0.0

        # For leg info callback (captain output)
        self.leg_course = 0.0
        self.leg_start_lat = 0.0
        self.leg_start_long = 0.0
        self.end_lat = 0.0
        self.end_long = 0.0

        self.publisher = rospy.Publisher("/speed_stats", SpeedStats, queue_size=10)

    def airmar_data_callback(self, data):
        self.lat = data.lat
        self.long = data.long
        self.truWndDir = data.truWindDir
        self.truWndSpd = data.truWindSpd
        self.cog = data.cog
        self.sog = data.sog

        self.publish()

    def leg_info_callback(self, data):
        self.leg_course = data.leg_course
        self.leg_start_lat = data.begin_lat
        self.leg_start_long = data.begin_long
        self.end_lat = data.end_lat
        self.end_long = data.end_long

        self.publish()

    def publish(self):
        msg = SpeedStats()
        msg.vmg = self.calc_vmg()
        msg.vmgup = self.calc_vmgup()
        msg.xte = self.calc_xte()

        self.publisher.publish(msg)

    # CALCULATION FUNCTIONS
    # returns the scalar projection of the vector (magnitude direction) onto course.
    @staticmethod
    def vector_projection(magnitude, direction, course):
        direction, course = list(map(radians, [direction, course]))
        return cos(direction - course) * magnitude

    # returns a (magnitude,angle) representation of the vector sum
    @staticmethod
    def vector_add(mag1, angle1, mag2, angle2):
        angle1, angle2 = list(map(radians, [angle1, angle2]))
        x = mag1 * cos(angle1) + mag2 * cos(angle2)
        y = mag1 * sin(angle1) + mag2 * sin(angle2)
        ret_mag = sqrt(x * x + y * y)
        ret_angle = (degrees(atan2(y, x))) % 360.0
        return ret_mag, ret_angle

    # calculate XTE from a target course and range given an intial course we wanted to sail from.
    # this will be negative when you are to the left of the leg_course vector (i.e. your t_course is
    # between 0 and 180 degrees and visa versa)
    @staticmethod
    def _calculate_xte(t_range, t_course, l_course):
        angle = (t_course - l_course) % 360
        angle = (-1 * angle) if (angle <= 180) else (360 - angle)
        xte = sin(radians(angle)) * t_range
        return xte

    def calc_vmg(self):
        return self.vector_projection(self.sog, self.cog, self.leg_course)

    def calc_vmgup(self):
        return -1.0 * self.vector_projection(self.sog, self.cog, self.truWndDir)  # Wind vector is in opposite direction of positive VMGup!!

    def calc_xte(self):
        target_course = gpsBearing(self.lat, self.long, self.end_lat, self.end_long)
        target_range = gpsDistance(self.lat, self.long, self.end_lat, self.end_long)
        return self._calculate_xte(target_range, target_course, self.leg_course)


if __name__ == '__main__':
    rospy.init_node('speed_calculator')
    spd = SpeedCalculator()
    rospy.Subscriber('/airmar_data', AirmarData, spd.airmar_data_callback)
    rospy.Subscriber('/leg_info', LegInfo, spd.leg_info_callback)
    rospy.loginfo('[speed_calculator] Started node')
    rospy.spin()

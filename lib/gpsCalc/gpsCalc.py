#!/usr/bin/env python

from math import sin, cos, asin, degrees, atan2, radians, degrees, sqrt, hypot
import utm
from geopy.distance import vincenty

#Returns the distance in m between two gps coordinates.

#FIXME
def gpsDistance(lat1, lon1, lat2, lon2):
  """
  Returns the distance in m between two gps coordinates.
  Could cause error if the two points are in different UTM zones
  """
  print vincenty((lat1, lon1), (lat2, lon2)).meters

#Returns the initial compass bearing in degrees from location 1 to location 2 along a great circle route
def gpsBearing(lat1, lon1, lat2, lon2):
  lon1, lat1, lon2, lat2 = list(map(radians, [lon1, lat1, lon2, lat2]))
  dlon = lon2 - lon1
  y = sin(dlon) * cos(lat2)
  x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
  d = degrees(atan2(y, x))
  return (d%360)

#Given a starting point and a vector (in meters), return the resulting gps point as a tuple
def gpsVectorOffset(lat1,lon1,brng,mag):
  brng = radians(brng)                    #FIXME THIS WILL CRASH IF WE CROSS UTM ZONES!!!!
  x,y,zn,zl  = utm.from_latlon(lat1, lon1)
  x += sin(brng) * mag
  y += cos(brng) * mag
  return utm.to_latlon(x,y,zn,zl)

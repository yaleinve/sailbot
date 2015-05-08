#!/usr/bin/env python

from math import sin, cos, asin, degrees, atan2, radians, degrees, sqrt, hypot
import utm

#Returns the distance in m between two gps coordinates.
def gpsDistance(lat1, lon1, lat2, lon2):
  """
  Returns the distance in m between two gps coordinates.
  """
  point1 = utm.from_latlon(lat1, lon1)
  point2 = utm.from_latlon(lat2, lon2)
  return hypot(point2[0] - point1[0], point2[1] - point1[1])


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


#  r = 6378100 # Radius of earth in meters.
#  d = mag/r # Magnitude as proportion of Earth's radius
#  retLat = asin(sin(lat1)*cos(d)+cos(lat1)*sin(d)*cos(brng))
#  retLon = lon1 + atan2(sin(brng)*sin(d)*cos(lat1),cos(d) - sin(lat1)*sin(retLat))
#  return (degrees(retLat),degrees(retLon))


#!/usr/bin/env python
#gpsCalc.py 						Eric Anderson + Andrew Malta Mar 2015
#A library of functions on gps coordinates.  Gps Lat and Long are each handled
#as float64's

from math import sin, cos, asin, degrees, atan2

#Returns the distance in m between two gps coordinates.
#~~Potential Bug: what if shortest path involves two compass directions (i.e. go over or near a poll)?
def gpsDistance(lat1, lon1, lat2, lon2):    
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6378100 # Radius of earth in meters.
    return c * r

#Returns the compass bearing in degrees from location 1 to location 2
def gpsBearing(lat1, lon1, lat2, lon2):
  dlon = lon2 - lon1
  y = sin(dlon) * cos(lat2)
  x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
  d = degrees(atan2(y, x))
  #ensure that d is between 0 and 360
  while(not ((d >= 0) and (d <= 360))):
    d += 360 * (d < 0) 
    
  return (d)


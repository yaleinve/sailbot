#!/usr/bin/env python
#gpsCalc.py 						Eric Anderson + Andrew Malta Mar 2015
#A library of functions on gps coordinates.  Gps Lat and Long are each handled
#as float64's

#Refer to http://www.movable-type.co.uk/scripts/latlong.html for math details

from math import sin, cos, asin, degrees, atan2, radians, degrees

#Returns the distance in m between two gps coordinates.
def gpsDistance(lat1, lon1, lat2, lon2):    
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = list(map(radians, [lon1, lat1, lon2, lat2]))

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6378100 # Radius of earth in meters.
    return c * r

#Returns the initial compass bearing in degrees from location 1 to location 2
def gpsBearing(lat1, lon1, lat2, lon2):
  lon1, lat1, lon2, lat2 = list(map(radians, [lon1, lat1, lon2, lat2]))
  dlon = lon2 - lon1
  y = sin(dlon) * cos(lat2)
  x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
  d = degrees(atan2(y, x))
  #ensure that d is between 0 and 360    #~~Eric commented this out- just use
  #while(not ((d >= 0) and (d <= 360))): #built in mod function?
  #  d += 360 * (d < 0) 
    
  return (d%360)

#Written by Eric 3/20/15
#Given a starting point and a vector (in meters), return the resulting gps point as a tuple
def gpsVectorOffset(lat1,lon1,brng,range):
  lat1,lon1,brng= list(map(radians,[lat1,lon1,brng]))  #Convert to radians
  r = 6378100 # Radius of earth in meters.
  d = range/r # Magnitude as proportion of Earth's radius
  retLat = asin(sin(lat1)*cos(d)+cos(lat1)*sin(d)*cos(brng))
  retLon = lon1 + atan2(sin(brng)*sin(d)*cos(lat1),cos(d) - sin(lat1)*sin(retLat))
  return (degrees(retLat),degrees(retLon))

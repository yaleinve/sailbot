#!/usr/bin/env python

from math import *
import utm
from geopy.distance import vincenty

#Returns the distance in m between two gps coordinates.

#FIXME
def gpsDistance(lat1, lon1, lat2, lon2):
  """
  Returns the distance in m between two gps coordinates.
  Could cause error if the two points are in different UTM zones
  """
  return vincenty((lat1, lon1), (lat2, lon2)).meters

#Returns the initial compass bearing in degrees from location 1 to location 2 along a great circle route
def gpsBearing(lat1, lon1, lat2, lon2):
  lon1, lat1, lon2, lat2 = list(map(radians, [lon1, lat1, lon2, lat2]))
  dlon = lon2 - lon1
  y = sin(dlon) * cos(lat2)
  x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
  d = degrees(atan2(y, x))
  return (d%360)

def destEllipse(lat1, lon1, brg, s, a, b):
  cs1 = ds1 = ss1 = cs1m = 0
  s *= 1000;
  ind = 1
  f = 1.0/298.257223563
  sb = sin(brg)
  cb = cos(brg)
  tu1 = (1-f) * tan(lat1)
  cu1 = 1 / sqrt(1 + tu1 * tu1)
  su1 = tu1 * cu1
  s2 = atan2(tu1, cb)
  sa = cu1 * sb
  csa = 1 - sa * sa
  us = csa * (a * a - b * b) / (b * b);
  A = 1 + us / 16384.0 * (4096 + us * (-768 + us * (320 - 175 * us)))
  B = us / 1024 * (256 + us * (-128 + us * (74 - 47 * us)))
  s1 = s / (b * A)
  s1p = 2 * pi
  while (fabs(s1 - s1p) > 1e-12):
    cs1m = cos(2 * s2 + s1)
    ss1 = sin(s1)
    cs1 = cos(s1)
    ds1 = B * ss1 * (cs1m + B / 4 * (cs1 * (-1 + 2 * cs1m * cs1m) - B / 6 * cs1m * (-3 + 4 * ss1 * ss1) * (-3 + 4 * cs1m * cs1m)))
    s1p = s1
    s1 = s / (b * A) + ds1
  t = su1 * ss1 - cu1 * cs1 * cb
  lat2 = atan2(su1 * cs1 + cu1 * ss1 * cb, (1 - f) * sqrt(sa * sa + t * t))
  l2 = atan2(ss1 * sb, cu1 * cs1 - su1 * ss1 * cb)
  c = f / 16 * csa * (4 + f * (4 - 3 * csa))
  l = l2 - (1 - c) * f * sa * (s1 + c * ss1 * (cs1m + c * cs1 * (-1 + 2 * cs1m * cs1m)))
  d = atan2(sa, -t)
  finalBrg = d + 2 * pi
  backBrg = d + pi
  newLon = lon1 + l
  return (finalBrg, backBrg, lat2, newLon)

#Given a starting point and a vector (in meters), return the resulting gps point as a tuple
def gpsVectorOffset(lat, lon, bearing, dist):
  mx = 41000.0
  radiusEarth = 6372.7976
  smaj = 6378137.0
  sminor = 6356752.314245179
  brg = [radians(bearing), 180, 0]
  lat = radians(lat)
  lon = radians(lon)
  finBrg, finBackBrg, finLat, finLon = destEllipse(lat, lon, brg[0], dist/1000.0, smaj, sminor)
  return (degrees(finLat), degrees(finLon))
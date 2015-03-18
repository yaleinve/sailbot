from math import radians, cos, sin, asin, sqrt, atan2, degrees

#returns the distance in km between two gps coordinates.
def gpsDistance(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    """
    
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles
    return c * r


def gpsBearing(lat1, lon1, lat2, lon2):
  dlon = lon2 - lon1
  y = sin(dlon) * cos(lat2)
  x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
  d = degrees(atan2(y, x))

  #ensure that d is between 0 and 360
  while(not ((d >= 0) and (d <= 360))):
    d += 360 * (d < 0) 
    
  return (d)

while True:
  lat1 = float(input("lat1 : "))
  lon1 = float(input("lon1 : "))
  lat2 = float(input("lat2 : "))
  lon2 = float(input("lon2 : "))

  print(gpsDistance(lat1, lon1, lat2, lon2))
  print(gpsBearing(lat1, lon1, lat2, lon2))

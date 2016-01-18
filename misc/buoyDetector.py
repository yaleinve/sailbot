import cv2
import numpy as np
import time
import math

#constants dependent on the camera. TODO update these constants.
OBJECT_HEIGHT = 1000.0 # object height in millimeters
FOCAL_LENGTH = 5.0 #focal length of the camera in mm
PIXELS_PER_MM = 600.0 #number of pixels per millimeter on the sensor


method = int(raw_input("1 for camera 2 for saved image: "))

img = None
if (method == 1):
    cam = cv2.VideoCapture(0)
    temp = time.time()
    while ((time.time() - temp) < 1):
        img = cam.read()[1]

elif (method == 2):
    img = cv2.imread(raw_input("name of the file in this working directory: "))

# Convert BGR to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# define range of the color of the buoy (Todo find the right HSV values)
lower_range = np.array([0,100,100])
upper_range = np.array([40,255,255])

# Threshold the HSV image to get only the buoy colors
mask = cv2.inRange(hsv, lower_range, upper_range)


# #draw the contours around the object.
contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(mask, contours, -1, (255,0,0), 1)


maxBoundingRectArea = 0.0
maxRect = None

if contours != []:
    for cnt in contours:
        x,y,w,h = cv2.boundingRect(cnt)
        if (w * h > maxBoundingRectArea):
            maxRect = (x,y,w,h)
            maxBoundingRectArea = w*h

    x, y, w, h = maxRect
    cv2.rectangle(mask, (x, y), (x+w, y+h), (255,255,255), 2)
    
    buoyHeightOnSensor = h / PIXELS_PER_MM
    distanceFromBouy = OBJECT_HEIGHT * (FOCAL_LENGTH / buoyHeightOnSensor) #distance we are away from the buoy in meters

    midpoint = len(img[0]) / 2.0 # the midpoint pixel along the x axis.
    pixelsOffCenter = (x + (w / 2.0)) - midpoint
    print "pixels off center " + str(pixelsOffCenter)
    
    distanceFromCenter = (pixelsOffCenter * OBJECT_HEIGHT) / h  # solve the proportion to find the distance in mm
    print "distance from center " + str(distanceFromCenter)

    bearing = math.degrees(math.asin(distanceFromCenter / distanceFromBouy))

    bearing %= 360

    print("you are " + str(distanceFromBouy / 1000.0) + " meters away from the buoy.")
    print("you are at a bearing of " + str(bearing) + " degrees from the buoy.")
    
while True:
    cv2.imshow('mask', mask)
    cv2.imshow('img', img)

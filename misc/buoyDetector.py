import cv2
import numpy as np
import time
import math

#constants dependent on the camera. TODO update these constants.
OBJECT_HEIGHT = 152.4 # object height in millimeters
FOCAL_LENGTH = 5.0 #focal length of the camera in mm
PIXELS_PER_MM = 600.0 #number of pixels per millimeter on the sensor
CIRCULARITY_EPSILON_WIDTH = .20 # constant for determining a cutoff of circularity.
CIRCULARITY_EPSILON_HEIGHT = .35 # constant for determining a cutoff of circularity.
MIN_AREA = 1000.0 # the minimum area in pixels that a buoy can be.

#determine if the contour passed in is circular (enough) to be a buoy.
def is_potential_buoy(cont):
    area = cv2.contourArea(cont)
    equi_diameter = np.sqrt(4*area/np.pi)
    x,y,w,h = cv2.boundingRect(cont)

    print "width: " + str(abs(equi_diameter - w) / w)

    if ((abs(equi_diameter - w) / w) < CIRCULARITY_EPSILON_WIDTH and
        (abs(equi_diameter - h) / h) < CIRCULARITY_EPSILON_HEIGHT and
        area >= MIN_AREA):
        print area
        return True
    return False


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

for i in range(2):
    maxBoundingRectArea = 0.0
    maxRect = None
    maxRectIndex = 0
    x,y,w,h = (0.0, 0.0, 0.0, 0.0)

    if contours != []:
        for j in range(len(contours)):
            if not is_potential_buoy(contours[j]):
                continue

            x,y,w,h = cv2.boundingRect(contours[j])
            if (w * h > maxBoundingRectArea):
                maxRect = contours[j]
                maxRectIndex = j
                maxBoundingRectArea = w*h

        if not maxRect is None:
            x, y, w, h = cv2.boundingRect(maxRect)
            cv2.rectangle(mask, (x, y), (x+w, y+h), (255,255,255), 2)
            
            buoyHeightOnSensor = h / PIXELS_PER_MM
            distanceFromBouy = OBJECT_HEIGHT * (FOCAL_LENGTH / buoyHeightOnSensor) #distance we are away from the buoy in meters

            midpoint = len(img[0]) / 2.0 # the midpoint pixel along the x axis.
            pixelsOffCenter = (x + (w / 2.0)) - midpoint
            
            distanceFromCenter = (pixelsOffCenter * OBJECT_HEIGHT) / h  # solve the proportion to find the distance in mm
            
            #TODO change method of calculating bearing. this method is off.
            bearing = math.degrees(math.asin(distanceFromCenter / distanceFromBouy))

            bearing %= 360

            print("Bouy #" + str(i + 1) + " you are " + str(distanceFromBouy / 1000.0) + " meters away from the buoy.")
            print("Bouy #" + str(i + 1) + " you are at a bearing of " + str(bearing) + " degrees from the buoy.")
            
            is_potential_buoy(maxRect)
            del contours[maxRectIndex]

while True:
    cv2.imshow('mask', mask)
    cv2.imshow('img', img)

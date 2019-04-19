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
#returns true if this contour is potentially a buoy.
#returns false otherwise.

class Contour():
    def __init__(self, cont):
        self.area = cv2.contourArea(cont)
        self.contour = cont
        self.x, self.y, self.w, self.h = cv2.boundingRect(cont)
    
    #returns true if the contour is a potential buoy. false otherwise.
    def is_potential_buoy(self):
        equi_diameter = np.sqrt(4*self.area/np.pi)

        if ((abs(equi_diameter - self.w) / self.w) < CIRCULARITY_EPSILON_WIDTH and
            (abs(equi_diameter - self.h) / self.h) < CIRCULARITY_EPSILON_HEIGHT and
            self.area >= MIN_AREA):
            return True
        return False

    #returns the distance from the buoy in meters.
    def calculate_distance(self):
        buoyHeightOnSensor = self.h / PIXELS_PER_MM
        return ((OBJECT_HEIGHT * (FOCAL_LENGTH / buoyHeightOnSensor)) / 1000.0)

    def calculate_bearing(self, img):
        # midpoint = len(img[0]) / 2.0 # the midpoint pixel along the x axis.
        # pixelsOffCenter = (self.x + (self.w / 2.0)) - midpoint
        # distanceFromCenter = (pixelsOffCenter * OBJECT_HEIGHT) / self.h  # solve the proportion to find the distance in mm
        
        # #TODO change method of calculating bearing. this method is off.
        # bearing = math.degrees(math.asin(distanceFromCenter / self.distance))

        return 0.0


    def __str__(self):
        return "This candidate buoy is " + str(self.distance) + " meters away at a bearing of " + str(self.bearing) + "."


def filter_for_color(img):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # define range of the color of the buoy (Todo find the right HSV values)
    lower_range = np.array([0,100,100])
    upper_range = np.array([40,255,255])

    # Threshold the HSV image to get only the buoy colors
    return cv2.inRange(hsv, lower_range, upper_range)


def detect_buoys(contours, mask):
    contourList = []

    for j in range(len(contours)):
        cont = Contour(contours[j])
        if not cont.is_potential_buoy():
            continue

        cont.distance = cont.calculate_distance()
        cont.bearing = cont.calculate_bearing(mask)
        contourList.append(cont)
        
    return contourList


if __name__ == "__main__":
    #code for demo ... Eventually to be deleted.
    # ------------------------------------------------------------
    method = int(raw_input("1 for camera 2 for saved image: "))

    img = None
    if (method == 1):
        cam = cv2.VideoCapture(0)
        temp = time.time()
        while ((time.time() - temp) < 1):
            img = cam.read()[1]

    elif (method == 2):
        img = cv2.imread(raw_input("name of the file in this working directory: "))
    # ------------------------------------------------------------

    mask = filter_for_color(img)

    #extract the contours from the masked image.
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    #draw the contours onto the masked image.
    cv2.drawContours(mask, contours, -1, (255,0,0), 1)

    candidate_buoys = detect_buoys(contours, mask)

    for c in candidate_buoys:
        cv2.rectangle(mask, (c.x, c.y), (c.x + c.w, c.y + c.h), (255,255,255), 2)
        print c

    while True:
        cv2.imshow('mask', mask)
        cv2.imshow('img', img)

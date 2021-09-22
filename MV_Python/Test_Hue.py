# -*- coding: utf-8 -*-
"""
Created on Tue Aug 31 16:20:28 2021

@author: Aleksei
"""

# Import of libraries
import numpy as np
from imutils.video import VideoStream
import cv2
from time import sleep

# Minimum and maximum detectable saturation and value (brightness)
S_MIN = 30
S_MAX = 255
V_MIN = 150
V_MAX = 255

# Empty function required to create trackbars
def nothing(x):
    pass

# Video stream object
vs = VideoStream(src=1).start()

# Delay for camera initialisation
sleep(2)

# Creation of window with sliders
# Creation of a window
cv2.namedWindow("Control")
# Resizing the window
cv2.resizeWindow("Control", 500, 100)
# Creating slider for hue
cv2.createTrackbar("Hue", "Control", 0, 180, nothing)
# Creating slider for deviation
cv2.createTrackbar("Deviation", "Control", 0, 20, nothing)

# Main code
# Starting an infinite loop
while True:
    
    # Reading an image from a video stream
    image = vs.read()
    
    # Getting the position of the deviation slider
    dev = cv2.getTrackbarPos("Deviation", "Control")
    
    # Getting the position of the hue slider and calculating the hue thresholds
    h_min = cv2.getTrackbarPos("Hue", "Control") - dev
    h_max = cv2.getTrackbarPos("Hue", "Control") + dev
    
    # Defining colour ranges in HSV
    lower_range = np.array([h_min, S_MIN, V_MIN])
    upper_range = np.array([h_max, S_MAX, V_MAX])
    
    # Converting the captured image to the HSV colour model
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Creating a mask for an HSV image according to the set hue
    thres = cv2.inRange(hsv, lower_range, upper_range)
    
    # Bitwise stacking of the original image and mask
    bitwise = cv2.bitwise_and(image, image, mask=thres)
    
    # Result output
    cv2.imshow("Hue Image", bitwise)
    
    # Breaking the infinite loop
    # Waits for a pressed key
    k = cv2.waitKey(1)
    # If ESC has been pressed
    if k == 27:
        # Breaking the loop
        break
# Closing all windows
cv2.destroyAllWindows()
# Stopping a video stream
vs.stop()
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 31 11:20:26 2021

@author: Aleksei
"""

# Import of libraries
from PIL import ImageDraw, ImageFont, Image
import numpy as np
from imutils.video import VideoStream
import imutils
import cv2
from time import sleep

# Creation of constants
# Font loading
FONT = ImageFont.truetype("/usr/share/fonts/truetype/freefont/TimesNewRomanBold.ttf", 18)

# Minimum registered spot area
BLOBSIZE = 600

# Minimum and maximum detectable saturation and value (brightness)
S_MIN = 30
S_MAX = 255
V_MIN = 150
V_MAX = 255

# Approximate values for hue filters
HUES = { 
        "Red": 3,
        "Orange": 15,
        "Yellow": 26, 
        "Green": 38,
        "Blue": 100,
        "Purple": 148, 
        }

# Assigning numbers to shapes
SHAPES = {
    '1': 'Triangle',
    '2': 'Square',
    '3': 'Rectangle',
    '4': 'Circle'
    }

# Assigning numbers to colours
COLOURS = {
    '1': 'Red',
    '2': 'Orange',
    '3': 'Yellow',
    '4': 'Green',
    '5': 'Blue',
    '6': 'Purple'
    }

# Requesting the required object shape from the user with input validation
while True:
    print("""\nWhat Shape Should Be Gripped? (Enter a Number)\n
          1 = Triangle,
          2 = Square,
          3 = Rectangle,
          4 = Circle\n""")
    user_shape = input('Shape: ')
    if user_shape in SHAPES.keys():
        break
    print("Please Enter a Correct Number!")
    
# Requesting the required object colour from the user with input validation
while True:
    print("""\nWhat Colour Should It Be? (Enter a Number)\n
          1 = Red,
          2 = Orange,
          3 = Yellow,
          4 = Green,
          5 = Blue,
          6 = Purple\n""")
    colour = input('Colour: ')
    if colour in COLOURS.keys():
        break
    print("Please Enter a Correct Number!")
    
# Displaying received data on screen
print('\nRequired Object: ' + COLOURS[colour] + ' ' + SHAPES[user_shape])

# Video stream object
vs = VideoStream(src=1).start()

# Delay for camera initialisation
sleep(2)


# Function for determining the number of angles of contour
def shapeDetect(c):
    
    # Initialisation of the shape name variable
    shape = ""
    # Calculating the perimeter of a closed curve
    peri = cv2.arcLength(c, True)
    # Determining the shape of contour using approximation and perimeter
    approx = cv2.approxPolyDP(c, 0.04 * peri, True)
    
    # Counting the number of vertices and defining a specific shape
    # If there are three vertices, it is a triangle
    if len(approx) == 3:
        shape = "Triangle"
        
    # If there are four vertices then it is necessary to calculate whether it is a rectangle or a square
    elif len(approx) == 4:
        # The rectangle into which the figure fits is calculated
        (x, y, w, h) = cv2.boundingRect(approx)
        # The ratio of the sides of this rectangle is determined
        ar = w / float(h)
        # If the aspect ratio is close to 1, it is a square
        if ar >= 0.7 and ar <= 1.3:
            shape = "Square"
        # Otherwise, it is a rectangle
        else:
            shape = "Rectangle"
            
    # Otherwise, it is a circle
    else: 
        shape = "Circle"
        
    # Return the name of the shape
    return shape


# Main code
# Starting an infinite loop
while True:
    
    # Main infinite loop code
    # Reading an image from a video stream
    image = vs.read()
    # Creating a copy of the image
    img_copy = image.copy()
    # Reducing image size to accelerate calculations
    resized = imutils.resize(image, width=300)
    # Calculating the ratio of the original image to the reduced image
    ratio = image.shape[0] / float(resized.shape[0])
    
    # Passing through all hue filters
    for hue in HUES:

        # Calculating the hue thresholds
        h_min = HUES[hue] - 6
        h_max = HUES[hue] + 6
        # Defining colour ranges in HSV
        lower_range = np.array([h_min, S_MIN, V_MIN])
        upper_range = np.array([h_max, S_MAX, V_MAX])
        
        # Converting the reduced image to the HSV colour model
        hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
        
        # Creating a mask for an HSV image from colour ranges
        thres = cv2.inRange(hsv, lower_range, upper_range)
        # Smoothing the mask to reduce high-frequency noise
        thres = cv2.GaussianBlur(thres, (5, 5), 0)
        
        # Displaying masks of all hues (uncomment to display)
        #cv2.imshow(hue, thres)
        
        # Finding contours in a mask
        cnts = cv2.findContours(thres.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Handling the grabbing of the correct tuple value
        cnts = imutils.grab_contours(cnts)
        
        # Passing through all found contours
        for c in cnts:
            
            # Checking the contour area
            # If the current contour area is small ...
            if cv2.contourArea(c) < BLOBSIZE:
                # Then ignore this contour and move on to the next one
                continue
            
            # Resizing the suitable contours to their original dimensions
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            
            # Displaying contours on a copy of the image
            # Colours of contours depend on the condition
            # Defining a contour shape using a function
            shapename = shapeDetect(c)
            # If the shape and hue of the object matches the gripping conditions ...
            if (shapename == SHAPES[user_shape]) and (hue == COLOURS[colour]):
                # Then draw a green contour
                cv2.drawContours(img_copy, [c], -1, (0, 255, 0), 2)
            else:
                # Otherwise, draw a red contour
                cv2.drawContours(img_copy, [c], -1, (0, 0, 255), 2)
                
        # Converting an image from BGR to RGB
        img = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB)
        # Creation of a PIL image from an array of pixels
        im_pil = Image.fromarray(img)
        # Creation of a drawing object
        draw = ImageDraw.Draw(im_pil)
        
        # Passing through all found contours again
        for c in cnts:
            
            # Checking the contour area again
            # If the current contour area is small ...
            if cv2.contourArea(c) < BLOBSIZE:
                # Then ignore this contour and move on to the next one
                continue
            
            # Calculating image moments for a suitable contour
            M = cv2.moments(c)
            cX = 0
            cY = 0
            # Calculating the coordinates of the centre of the contour
            if M["m00"] != 0:
                cX = int((M["m10"] / M["m00"]) * ratio)
                cY = int((M["m01"] / M["m00"]) * ratio)
                
            # Displaying object names on a copy of the image
            # Defining a contour shape using the function again
            shapename = shapeDetect(c)
            # If the shape and hue of the object matches the gripping conditions ...
            if (shapename == SHAPES[user_shape]) and (hue == COLOURS[colour]):
                # Then "[GRIP]" should be in the name
                grip = "\n[GRIP]"
            else:
                # Otherwise,"[NO GRIP]" should be in the name
                grip = "\n[NO GRIP]"
            # Creating the final title from hue, shape and grip names
            shapename = hue + " " + shapename + " " + grip
            # Displaying the final name in the drawing object
            draw.text((cX, cY), shapename, font=FONT)
        # Conversion of a PIL image object back to an array of pixels
        img_copy = np.asarray(im_pil)
        # Conversion of an image back from RGB to BGR
        img_copy = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB)
            
    # Image output
    cv2.imshow("Image", img_copy)
    
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
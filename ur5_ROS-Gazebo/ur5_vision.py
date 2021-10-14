#!/usr/bin/env python

"""
    moveit_cartesian_path.py - Version 0.1 2016-07-28

    Based on the R. Patrick Goebel's moveit_cartesian_demo.py demo code.

    Plan and execute a Cartesian path for the end-effector.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""
####################### MV Libraries
from PIL import ImageDraw, ImageFont, Image
#######################
import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
import geometry_msgs.msg
from ur5_notebook.msg import Tracker
import moveit_msgs.msg
import cv2, cv_bridge
from sensor_msgs.msg import Image


from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
tracker = Tracker()


####################### MV Const
# Font loading
#FONT = ImageFont.truetype("/usr/share/fonts/truetype/freefont/TimesNewRomanBold.ttf", 18)

# Minimum registered spot area
BLOBSIZE = 600

# Minimum and maximum detectable saturation and value (brightness)
S_MIN = 225
S_MAX = 255
V_MIN = 225
V_MAX = 255

# Approximate values for hue filters
HUES = { 
        "Red": 3,
        "Orange": 16,
        "Yellow": 26, 
        "Green": 55,
        "Blue": 120,
        "Purple": 130, 
        }

# Assigning numbers to shapes
SHAPES = {
    	1: 'Triangle',
    	2: 'Square',
    	3: 'Rectangle',
    	4: 'Circle'
    	}

# Assigning numbers to colours
COLOURS = {
    	1: 'Red',
    	2: 'Orange',
    	3: 'Yellow',
    	4: 'Green',
    	5: 'Blue',
    	6: 'Purple'
    	}
#######################


####################### MV Request
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

colour_value = COLOURS[colour]
colour_value = HUES[colour_value]

#######################


class ur5_vision:


####################### SHAPE DETECT
# Function for determining the number of angles of contour
    #def shapeDetect(cnts):



#######################

    def __init__(self):
        rospy.init_node("ur5_vision", anonymous=False)
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/ur5/usbcam/image_raw', Image, self.image_callback)
        self.cxy_pub = rospy.Publisher('cxy', Tracker, queue_size=1)


    def image_callback(self,msg):

        # BEGIN BRIDGE
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        # END BRIDGE
        # BEGIN HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # END HSV


####################### CONTOUR DETECT
	# Passing through all hue filters
    	for hue in HUES:

		# Calculating the hue thresholds
        	h_min = HUES[hue] - 6
        	h_max = HUES[hue] + 6
        	# Defining colour ranges in HSV
        	lower_range = np.array([h_min, S_MIN, V_MIN])
        	upper_range = np.array([h_max, S_MAX, V_MAX])

		# Creating a mask for an HSV image from colour ranges (NEED ???)
       		thres = cv2.inRange(hsv, lower_range, upper_range)
        	# Smoothing the mask to reduce high-frequency noise (NEED ???)
        	thres = cv2.GaussianBlur(thres, (5, 5), 0)

       		# Finding contours in a mask
        	(_, cnts, _) = cv2.findContours(thres.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		# Calculating image moments for a suitable contour
		M = cv2.moments(thres)
		if M['m00'] > 0:
            		cx = int(M['m10']/M['m00'])
            		cy = int(M['m01']/M['m00'])


        	# Handling the grabbing of the correct tuple value
       		# cnts = imutils.grab_contours(cnts)

		# Passing through all found contours
        	for i, c in enumerate(cnts):

			# Checking the contour area
            		# If the current contour area is small ...
			area = cv2.contourArea(c)
            		if area < 7500:
                		# Then ignore this contour and move on to the next one
                		continue


##################################################### SHAPE DETECT START
			# Displaying contours on the image
            		# Colours of contours depend on the condition
            		# Defining a contour shape

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
        			if ar >= 0.95 and ar <= 1.05:
            				shape = "Square"
        			# Otherwise, it is a rectangle
        			else:
            				shape = "Rectangle"
            
    			# Otherwise, it is a circle
    			else: 
        			shape = "Circle"
        
    			# Return the name of the shape
    			#return shape


##################################################### SHAPE DETECT END
        		# If the shape and hue of the object matches the gripping conditions ...
           		if (shape == SHAPES[user_shape]) and (hue == COLOURS[colour]):
               			# Then draw a green contour
               			cv2.drawContours(image, cnts, -1, (0, 255, 0),5)
				# Then "[GRIP]" should be in the name
                		grip = "[GRIP]"
            		else:
           			# Otherwise, draw a red contour
            			cv2.drawContours(image, cnts, -1, (0, 0, 255),5)
				# Otherwise,"[NO GRIP]" should be in the name
                		grip = "[NO GRIP]"


			# Creating the final title from hue, shape and grip names
            		name = hue + " " + shape

			cv2.putText(image, name, (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
			cv2.putText(image, grip, (cx+60, cy+40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

####################### "Gripping Algorithm" START


        # BEGIN FILTER
        lower_hue = np.array([colour_value - 6,  100, 100])
        upper_hue = np.array([colour_value + 6, 255, 255])
        mask = cv2.inRange(hsv, lower_hue, upper_hue)
        (_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #area = cv2.contourArea(cnts)
        h, w, d = image.shape
        # print h, w, d  (800,800,3)
        #BEGIN FINDER
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

        # cx range (55,750) cy range( 55, ~ )
        # END FINDER
        # Isolate largest contour
        #  contour_sizes = [(cv2.contourArea(contour), contour) for contour in cnts]
        #  biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
            for i, c in enumerate(cnts):
                area = cv2.contourArea(c)

##################################################### 2 SHAPE DETECT START
		# Displaying contours on the image
            	# Colours of contours depend on the condition
           	# Defining a contour shape

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
        		(x2, y2, w2, h2) = cv2.boundingRect(approx)
        		# The ratio of the sides of this rectangle is determined
        		ar = w2 / float(h2)
        		# If the aspect ratio is close to 1, it is a square
        		if ar >= 0.95 and ar <= 1.05:
            			shape = "Square"
        		# Otherwise, it is a rectangle
        		else:
            			shape = "Rectangle"
            
    		# Otherwise, it is a circle
    		else: 
        		shape = "Circle"
        

##################################################### 2 SHAPE DETECT END
		
		#
		#if SHAPES[user_shape] == "Square":
		#	 SHAPES[user_shape] = "Rectangle"
		#if shape == "Square":
		#	 shape = "Rectangle"


                if (area > 7500) and (shape == SHAPES[user_shape]):
                    self.track_flag = True
                    self.cx = cx
                    self.cy = cy
                    self.error_x = self.cx - w/2
                    self.error_y = self.cy - (h/2+195)
                    tracker.x = cx
                    tracker.y = cy
                    tracker.flag1 = self.track_flag
                    tracker.error_x = self.error_x
                    tracker.error_y = self.error_y
                    #(_,_,w_b,h_b)=cv2.boundingRect(c)
                    #print w_b,h_b
                    # BEGIN circle
                    cv2.circle(image, (cx, cy), 5, (100,100,100), -1)
                    cv2.putText(image, "({}, {})".format(int(cx), int(cy)), (int(cx-140), int(cy-20)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (100, 100, 100), 2)
                    cv2.drawContours(image, cnts, -1, (255, 255, 255),1)
                    #BGIN CONTROL
                    break
                else:
                    self.track_flag = False
                    tracker.flag1 = self.track_flag

####################### "Gripping Algorithm" END

        self.cxy_pub.publish(tracker)
        cv2.namedWindow("Camera", 1)
        cv2.imshow("Camera", image )
        cv2.waitKey(1)

follower=ur5_vision()
rospy.spin()

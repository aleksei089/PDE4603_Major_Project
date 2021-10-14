#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import of libraries
import rospy

# MV Const
# Font loading
#FONT = ImageFont.truetype("/usr/share/fonts/truetype/freefont/TimesNewRomanBold.ttf", 18)

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


# MV Request
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
print(colour_value)
#######################

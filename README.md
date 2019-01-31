# ROS-Camera
Using Camera in ROS


How to find functions by name in OpenCV
1.  Dumping all OpenCV function names and attributes:
>>import cv2
>>funcs = dir(cv2)
>>for f in funcs:
>>print(f)

2.  Searching the OpenCV library for (partial) function names
>>> import imutils
>>> imutils.find_function("contour")

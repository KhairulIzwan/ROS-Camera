# ROS-Camera
Using Camera in ROS

Important Notes:

1.  How to find functions by name in OpenCV
a.  Dumping all OpenCV function names and attributes:
$import cv2
$funcs = dir(cv2)
$for f in funcs:
$print(f)

b.  Searching the OpenCV library for (partial) function names
$ import imutils
$ imutils.find_function("contour")

2.  Changes package from uvc_camera to cv_camera for usb-camera, webcam, and etc

Files:
1. camera.launch (Updated --01022019 -- 4.22am (Ubuntu))
  --  launch file of camera based on cv_camera parameter setting
  --  for single camera (mono)

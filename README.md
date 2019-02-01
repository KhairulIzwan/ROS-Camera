# ROS-Camera
Using Camera in ROS

## Important Notes:

### How to find functions by name in OpenCV

1.  Dumping all OpenCV function names and attributes:
    1.  import cv2
    2.  funcs = dir(cv2)
    3.  for f in funcs:
    4.  print(f)

2.  Searching the OpenCV library for (partial) function names
    1.  import imutils
    2.  imutils.find_function("contour")

### uvc_camera --> cv_camera package
3.  Changes package from uvc_camera to cv_camera for usb-camera, webcam, and etc

### Files:
1.  camera.launch (Updated --01022019 -- 4.22am (Ubuntu))
    1.  launch file of camera based on cv_camera parameter setting
    2.  for single camera (mono)
    3.  Deleting comments

2.  double_camera.launch (Update -- 010122019 -- 4.42pm (Ubuntu))
    1.  launch file of camera based on cv_camera parameter setting
    2.  for multiple camera (mono)

3.  test_vision_node.py (Update -- 010122019 -- 5.37pm (Ubuntu))
    1.  subscribe with camera0/cam0

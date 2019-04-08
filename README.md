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

2.  double_camera.launch (Update -- 01012019 -- 4.42pm (Ubuntu))
    1.  launch file of camera based on cv_camera parameter setting
    2.  for multiple camera (mono)

3.  test_vision_node.py (Update -- 01012019 -- 5.37pm (Ubuntu))
    1.  subscribe with camera0/cam0
    2.  New:
        1.  Updating the way we subscribed, and published a topics (Update -- 08042019 -- 5.37pm (Ubuntu))

4.  camera_opencv.launch (Update -- 01022019 -- 6.51pm (Ubuntu))
    1.  launch cv_camera (mono) + test_vision_node.py

5.  tracking_node.py (Update -- 01022019 -- 6.51pm (Ubuntu))
    1.  tracking greencolor object (predefined; hardcoded)

6.  camera_opencv_tracking.launch (Update -- 01022019 -- 6.51pm (Ubuntu))
    1.  launch cv_camera (mono) + tracking_node.py

7.  tracking_movement_node.py (Update -- 01022019 -- 7.10pm (Ubuntu))
    1.  the continuity of tracking greencolor object (predefined; hardcoded); tracking_node.py
    2.  points (pts) of tracking points are accumulated and drawn -- in conjuntion can identify the movement of object tracking -- East, West, North, and etc.

8.  object_tracking_node (Update -- 01042019 -- 12.15am (Ubuntu))
    1.  8 OpenCV Object Tracking Implementations

9.  range_detector_node (Update -- 08042019 -- (Ubuntu))
    1.  Using trackbar (color) to fine tune color filter (HSV or RGB)

#### All codes (python -- object tracking etc were originally comes from www.pyimagesearch.com; some modification has been made to make it possible to works in ROS environment)

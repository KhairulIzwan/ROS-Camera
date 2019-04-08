# ROS-Recap
1.  The core entity in ROS is called a node. A node is generally:
    1.  a small program written in Python or C++ that executes some relatively simple task or process.
    2.  can be started and stopped independently of one another and they communicate by passing messages.
    3.  can publish messages on certain topics or provide services to other nodes.
2.  http://wiki.ros.org/ROS/Tutorials

# Building a PiRobot
## Camera Node
1.  cv_camera uses OpenCV capture object to capture camera image
    1.  http://wiki.ros.org/cv_camera
    2.  http://wiki.ros.org/dynamic_reconfigure
    3.  if use more than one camera:
        1.  https://answers.ros.org/question/288227/how-to-view-multiple-cameras-sing-cv_camera/
        2.  Eg:
            1.  rosrun cv_camera cv_camera_node _device_id:=0 __name:=cam1
            2.  rosrun cv_camera cv_camera_node _device_id:=1 __name:=cam2
        3.  change the device_id (mandatory) and name
    4. roslaunch camera_tutorials camera.launch (easiest way)
    5. Publish: from sensor_msgs.msg import Image

## Range Detector Node
1.  use color filter (RGB or HSV) to find the upper and lower color range
2.  rosrun camera_tutorials range_detector_node.py HSV (or RGB)
3.  Publish: from camera_tutorials.msg import IntList
4.  Subscribe: from sensor_msgs.msg import Image
5.  Publish with unique name (anonymous=True)

## Ball Tracking Node
1.  rosrun camera_tutorials ball_tracking_node.py  
2.  Subscribe: from camera_tutorials.msg import IntList
3.  Publish: from sensor_msgs.msg import RegionOfInterest

# How to install packages from github
1.  cs OR cd ~/catkin_ws/src
2.  git clone -b <branch> <address>
    1. example: git clone -b https://github.com/KhairulIzwan/ROS-Camera.git
3.  cm OR cd ~/catkin_ws && catkin_make
4.  rospack profile (refresh the package available)

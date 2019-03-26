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
        2.  change the device_id (mandatory) and name

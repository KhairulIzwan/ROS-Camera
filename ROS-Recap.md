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
    3.  <launch>
          <node ns="cv_camera_node" pkg="cv_camera" name="cam0" type="cv_camera_node" output="screen">
            <param name="rate" value="30" />
            <param name="device_id" value="0" />
            <param name="frame_id" value="camera_primer" />
            <param name="image_width" value="320" />
            <param name="image_height" value="240" />
            <!-- <param name="camera_info" value="???" /> -->
            <!-- <param name="file" value="???" /> -->
            </node>

          <node name="dynamic_reconfigure" pkg="dynamic_reconfigure" type="reconfigure_gui" />
        </launch>
    4.  if use more than one camera:
        1.  https://answers.ros.org/question/288227/how-to-view-multiple-cameras-using-cv_camera/
        2.  change the device_id (mandatory) and name

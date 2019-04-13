# Setup
##  PC Setup (or Remote PC)
1.  Install Ubuntu on Remote PC:
    1.  https://www.ubuntu.com/download/alternative-downloads
    2.  https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop#0

2.  Install ROS on Remote PC:
    1.  sudo apt-get update
    2.  sudo apt-get upgrade
    3.  wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh

3.  Install Dependent ROS Packages:
    1.  sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
    2.  cd ~/catkin_ws/src/
    3.  git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
    4.  git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
    5.  cd ~/catkin_ws && catkin_make

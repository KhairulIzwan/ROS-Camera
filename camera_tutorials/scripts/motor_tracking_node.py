#!/usr/bin/env python

# motor tracking

from __future__ import print_function
import roslib
roslib.load_manifest('camera_tutorials')
import sys
import rospy
import cv2
import numpy as np
import imutils
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import CameraInfo

from camera_tutorials.msg import IntList
from camera_tutorials.msg import detailROI

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

from collections import deque
"""  a list-like data structure with super fast appends and pops to maintain a list of the past N (x, y)-locations of the ball in our video stream. Maintaining such a queue allows us to draw the "contrail" of the ball as its being tracked """

import os

class motor_tracking_node:
    def __init__(self):

        """  Initializing your ROS Node """
        rospy.init_node('motor_tracking_node', anonymous=True)

        """  Give the OpenCV display window a name """
        self.cv_window_original = "Target Ball"

        """  Give the camera driver a moment to come up """
        rospy.sleep(1)

        """  Create the cv_bridge object """
        self.bridge = CvBridge()

        """  Subscribe to the raw camera image topic """
        self.imgRaw_sub = rospy.Subscriber("/cam0/image_raw", Image, self.callback)

        """  Subscribe to the info camera topic """
        self.imgInfo_sub = rospy.Subscriber("/cam0/camera_info", CameraInfo, self.getCameraInfo)

        """  Subscribe to the green roi topic """
        self.imgROI_sub = rospy.Subscriber("/roi_Green", detailROI, self.getGreenROI)

        """  Subscribe to the red roi topic """
        self.imgROI_sub = rospy.Subscriber("/roi_Red", detailROI, self.getRedROI)

    """ Processing image raw """
    def callback(self, msg):
        """ Convert the raw image to OpenCV format using the cvtImage() helper function """
        self.cvtImage(msg)

        """ Apply image processing using imgProcessing() helper function """
        self.imgProcessing()

        """ Search for biggest (nearest) colored object using colorDetection() helper function """
        # self.ballTrack()

        """ Refresh the displayed image """
        self.displayImg()

    """ Getting camera info -- width, height, and etc """
    def getCameraInfo(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height

    """ Getting green colored ROI info -- width, height, and etc """
    def getGreenROI(self, msg):
        self.colorName_Green = msg.colorName
        self.offsetX_Green = msg.offsetX
        self.offsetY_Green = msg.offsetY
        self.width_Green = msg.width
        self.height_Green = msg.height
        self.x_Green = msg.x
        self.y_Green = msg.y
        self.radius_Green = msg.radius

        self.ballGreenTrack()

    """ Getting green colored ROI info -- width, height, and etc """
    def getRedROI(self, msg):
        self.colorName_Red = msg.colorName
        self.offsetX_Red = msg.offsetX
        self.offsetY_Red = msg.offsetY
        self.width_Red = msg.width
        self.height_Red = msg.height
        self.x_Red = msg.x
        self.y_Red = msg.y
        self.radius_Red = msg.radius

        self.ballRedTrack()

    def ballGreenTrack(self):
        if self.colorName_Green is not None:
            rospy.loginfo(self.colorName_Green)

    def ballRedTrack(self):
        if self.colorName_Red is not None:
            rospy.loginfo(self.colorName_Red)

    """ coverting the ROS image data to uint8 OpenCV format """
    def cvtImage(self, ros_image):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            self.cv_image_copy = self.cv_image.copy()
        except CvBridgeError as e:
            print(e)

    """ Do basic image processing """
    def imgProcessing(self):
        if (self.image_width > 320):
            self.cv_image = imutils.resize(self.cv_image, width = 320)
        else:
            pass

        """ optional -- image-mirrored """
        # self.cv_image = cv2.flip(self.cv_image, 1)

    """ Refresh the displayed image """
    def displayImg(self):
        if self.colorName_Green is not None:
            # rospy.loginfo(self.colorName_Green)
            cv2.circle(self.cv_image_copy, (int(self.x_Green), int(self.y_Green)), int(self.radius_Green), (0, 255, 0), 2)
            cv2.imshow(self.cv_window_original, self.cv_image_copy)
        else:
            cv2.imshow(self.cv_window_original, self.cv_image)

        cv2.waitKey(1)

def usage():
    print("%s" % sys.argv[0])

def main(args):
    vn = motor_tracking_node()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Motor tracking node [OFFLINE]...")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("Motor tracking node [ONLINE]...")
        main(sys.argv)
    else:
        print(usage())
        sys.exit(1)

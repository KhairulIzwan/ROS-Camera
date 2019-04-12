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
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from camera_tutorials.msg import IntList, detailROI
from cv_bridge import CvBridge, CvBridgeError

from collections import deque
"""  a list-like data structure with super fast appends and pops to maintain a list of the past N (x, y)-locations of the ball in our video stream. Maintaining such a queue allows us to draw the "contrail" of the ball as its being tracked """

import os

class Tracking:
    def __init__(self):
        """ Getting camera info -- width, height, and etc """
        self.image_width = None
        self.image_height = None

        """ Getting green colored ROI info -- width, height, and etc """
        self.colorName_Green = None
        self.offsetX_Green = None
        self.offsetY_Green = None
        self.width_Green = None
        self.height_Green = None
        self.x_Green = None
        self.y_Green = None
        self.radius_Green = None

        """ Getting green colored ROI info -- width, height, and etc """
        self.colorName_Red = None
        self.offsetX_Red = None
        self.offsetY_Red = None
        self.width_Red = None
        self.height_Red = None
        self.x_Red = None
        self.y_Red = None
        self.radius_Red = None

        # """  Initializing your ROS Node """
        # """  rospy.init_node('my_node_name', anonymous=True) or rospy.init_node('my_node_name') """
        # rospy.init_node('motor_tracking_node', anonymous=True)

        """  Give the OpenCV display window a name """
        self.cv_window_original = "Target Ball"

        # """  rospy.Publisher initialization """
        # """  pub = rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10) """
        # """  The only required arguments to create a rospy.Publisher are the topic name, the Message class, and the queue_size """
        #
        # """  Publish roi topic """
        # # self.imgROI_pub = rospy.Publisher("/roi_" + self.names, detailROI, queue_size=10)

        """  Give the camera driver a moment to come up """
        rospy.sleep(1)

        """  Create the cv_bridge object """
        self.bridge = CvBridge()

        # """  subscribe to a topic using rospy.Subscriber class """
        # """  sub = rospy.Subscriber('TOPIC_NAME', TOPIC_MESSAGE_TYPE, name_callback) """
        #
        # """  Subscribe to the raw camera image topic """
        # self.imgRaw_sub = rospy.Subscriber("/cam0/image_raw", Image, self.callback)

        # """  Subscribe to the info camera topic """
        # self.imgInfo_sub = rospy.Subscriber("/cam0/camera_info", CameraInfo, self.getCameraInfo)

        # """  Subscribe to the green roi topic """
        # self.imgInfo_sub = rospy.Subscriber("/roi_Green", detailROI, self.getGreenROI)

        # """  Subscribe to the red roi topic """
        # self.imgInfo_sub = rospy.Subscriber("/roi_Red", detailROI, self.getRedROI)

    """ Processing image raw """
    def getCameraImage(self, msg):
        """ Convert the raw image to OpenCV format using the cvtImage() helper function """
        self.cvtImage(msg)

        """ Apply image processing using imgProcessing() helper function """
        self.imgProcessing()

        """ Search for biggest (nearest) colored object using colorDetection() helper function """
        self.ballTrack()
        rospy.loginfo(self.colorName_Green)

        """ OPTIONAL """
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

        # self.ballTrack()

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

        # self.ballTrack()

    def ballTrack(self):
        if self.colorName_Green:
            cv2.putText(self.cv_image, self.colorName_Green + " ball", (int(self.x_Green - self.radius_Green),int(self.y_Green - self.radius_Green)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(self.cv_image, " ", (int(self.x_Green - self.radius_Green),int(self.y_Green - self.radius_Green)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def displayImg(self):
        """ OPTIONAL """
        # TODO: How to display the bigger roi only
        # if not self.colorName_Green:
        #     cv2.circle(self.cv_image, (int(0), int(0)), int(0), (0, 255, 0), 1)
        # else:
        #     cv2.circle(self.cv_image, (int(self.x_Green), int(self.y_Green)), int(self.radius_Green), (0, 255, 0), 2)

        """ Refresh the displayed image """
        cv2.imshow(self.cv_window_original, self.cv_image)
        cv2.waitKey(1)

    def cvtImage(self, ros_image):
        """ coverting the ROS image data to uint8 OpenCV format """
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            self.cv_image_copy = self.cv_image.copy()

        except CvBridgeError as e:
            print(e)

    def imgProcessing(self):
        """ resize the frame, blur it, and convert it to the HSV color space """
        if (self.image_width > 320):
            self.cv_image = imutils.resize(self.cv_image, width = 320)
        else:
            pass

        """ optional -- image-mirrored """
        # self.cv_image = cv2.flip(self.cv_image, 1)

        # self.blurred = cv2.GaussianBlur(self.cv_image, (11, 11), 0)
        # self.hsv = cv2.cvtColor(self.blurred, cv2.COLOR_BGR2HSV)



# def usage():
#     print("Please specify:")
#     print("%s" % sys.argv[0])
#
# def main(args):
#     vn = motor_tracking_node()
#
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print("Shutting down")
#
#     cv2.destroyAllWindows()
#
# if __name__ == '__main__':
#     if len(sys.argv) == 1:
#         main(sys.argv)
#     else:
#         print(usage())
#         sys.exit(1)

if __name__ == '__main__':
    """  Initializing your ROS Node """
    """  rospy.init_node('my_node_name', anonymous=True) or rospy.init_node('my_node_name') """
    rospy.init_node('motor_tracking_node', anonymous=True)

    """ # TODO: """
    tracking = Tracking()

    """  subscribe to a topic using rospy.Subscriber class """
    """  sub = rospy.Subscriber('TOPIC_NAME', TOPIC_MESSAGE_TYPE, name_callback) """

    """  Subscribe to the raw camera image topic """
    rospy.Subscriber("/cam0/image_raw", Image, tracking.getCameraImage)

    """  Subscribe to the info camera topic """
    rospy.Subscriber("/cam0/camera_info", CameraInfo, tracking.getCameraInfo)

    """  Subscribe to the green roi topic """
    rospy.Subscriber("/roi_Green", detailROI, tracking.getGreenROI)

    """  Subscribe to the red roi topic """
    rospy.Subscriber("/roi_Red", detailROI, tracking.getRedROI)

    rospy.spin()

#!/usr/bin/env python

""" UPDATED VERSION OF test_vision_node.py """

from __future__ import print_function
import roslib
roslib.load_manifest('camera_tutorials')

import sys
import rospy
import cv2
import imutils

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

class test_vision_node:
    def __init__(self):

        """  Initializing your ROS Node """
        rospy.init_node('test_vision_node', anonymous=True)

        """ Give the OpenCV display window a name """
        self.cv_window_name = "OpenCV Image"

        """ Create the cv_bridge object """
        self.bridge = CvBridge()

        """ Subscribe to the raw camera image topic """
        self.imgRaw_sub = rospy.Subscriber("/cam0/image_raw", Image, self.callback)

    def callback(self,data):
        try:
            """ Convert the raw image to OpenCV format """
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)

        # TODO: can directly get by subscribing to camera info topics
        """ Get the width and height of the image """
        (height, width, channels) = cv_image.shape    # rows, columns and channels

        """ Overlay some text onto the image display """
        img = cv_image

        text = "OpenCV Image"
        org = (10, height - 10)

        fontFace = cv2.FONT_HERSHEY_DUPLEX
        fontScale = 0.5
        color = (255, 255, 255)
        thickness = 1
        lineType = cv2.LINE_AA
        bottomLeftOrigin = False # if True (text upside down)

        text1 = "(%d, %d)" % (width, height)
        org1 = (width - 100, height - 10)

        cv2.putText(img, text, org, fontFace, fontScale, color, thickness, lineType, bottomLeftOrigin)
        cv2.putText(img, text1, org1, fontFace, fontScale, color, thickness, lineType, bottomLeftOrigin)

        """ Refresh the image on the screen """
        cv2.imshow(self.cv_window_name, cv_image)
        cv2.waitKey(1)

def usage():
    print("%s" % sys.argv[0])

def main(args):
    vn = test_vision_node()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Test vision node [OFFLINE]...")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("Test vision node [ONLINE]...")
        main(sys.argv)
    else:
        print(usage())
        sys.exit(1)

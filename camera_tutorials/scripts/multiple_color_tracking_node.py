#!/usr/bin/env python

# multiple color tracking (color) -- useful for display

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
from camera_tutorials.msg import IntList
from cv_bridge import CvBridge, CvBridgeError

from collections import deque
"""  a list-like data structure with super fast appends and pops to maintain a list of the past N (x, y)-locations of the ball in our video stream. Maintaining such a queue allows us to draw the "contrail" of the ball as its being tracked """

import os

class multiple_color_tracking_node:
    def __init__(self):
        """  Initializing your ROS Node """
        """  rospy.init_node('my_node_name', anonymous=True) """
        """  or """
        """  rospy.init_node('my_node_name') """
        rospy.init_node('multiple_color_tracking_node', anonymous=True)

        """  Give the OpenCV display window a name """
        self.cv_window_original = "OpenCV Image"
        self.cv_window_mask = "OpenCV Image Mask"
        self.cv_window_target = "Target"

        """  rospy.Publisher initialization """
        """  pub = rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10) """
        """  The only required arguments to create a rospy.Publisher are the topic name, the Message class, and the queue_size """
        self.roi_pub = rospy.Publisher("/roi", RegionOfInterest, queue_size=10)

        """  Give the camera driver a moment to come up """
        rospy.sleep(1)

        """  Create the cv_bridge object """
        self.bridge = CvBridge()

        """  subscribe to a topic using rospy.Subscriber class """
        """  sub = rospy.Subscriber('TOPIC_NAME', TOPIC_MESSAGE_TYPE, name_callback) """

        """  Subscribe to the raw camera image topic """
        self.img_sub = rospy.Subscriber("/cam0/image_raw", Image, self.callback)

        """  Subscribe to the info camera topic """
        self.imgInfo_sub = rospy.Subscriber("/cam0/camera_info", CameraInfo, self.getCameraInfo)

        # TODO: Need to replace this; read from files
        # TODO: Red, Black, Blue, Yellow, White -- Use range_detector_node (hardcoded)
        """  Color Range (Upper and Lower) """
        """ define the lower and upper boundaries of the colors in the HSV color space """
        self.lower = {'red':(134, 151, 36), 'green':(12, 59, 50)}
        self.upper = {'red':(196, 248, 156), 'green':(53, 204, 216)}

        """ define standard colors for circle around the object """
        self.colors = {'red':(0,0,255), 'green':(0,255,0)}

    def callback(self, data):
        """ Convert the raw image to OpenCV format using the cvtImage() helper function """
        self.cvtImage(data)

        """ Apply image processing using imgProcessing() helper function """
        self.imgProcessing()

        """ Apply ball tracking using colorDetection() helper function """
        self.colorDetection()

        """Un-comment to get tracking echo"""
        # self.do_trackPts()

        """ Refresh the displayed image """
        cv2.imshow(self.cv_window_original, self.cv_image)
        # cv2.imshow(self.cv_window_target, self.cv_image_target)
        # cv2.imshow(self.cv_window_mask, self.cv_mask)
        cv2.waitKey(1)

    def cvtImage(self, ros_image):
        """ coverting the ROS image data to uint8 OpenCV format """
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            self.cv_image_copy = self.cv_image.copy()

        except CvBridgeError as e:
            print(e)

    def getCameraInfo(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height

    def imgProcessing(self):
        """ resize the frame, blur it, and convert it to the HSV color space """
        if (self.image_width > 320):
            self.cv_image = imutils.resize(self.cv_image, width = 320)
        else:
            pass

        """ optional -- image-mirrored """
        # self.cv_image = cv2.flip(self.cv_image, 1)

        self.blurred = cv2.GaussianBlur(self.cv_image, (11, 11), 0)
        self.hsv = cv2.cvtColor(self.blurred, cv2.COLOR_BGR2HSV)

    def colorDetection(self):

        """ for each color in dictionary check object in frame """
        for self.key, self.value in self.upper.items():

            """ construct a mask for the color from dictionary`1, then perform a series of dilations and erosions to remove any small blobs left in the mask """
            # self.kernel = np.ones((9,9), np.uint8)
            # self.mask = cv2.inRange(self.hsv, self.lower[self.key], self.upper[self.key])
            # self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_OPEN, self.kernel)
            # self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, self.kernel)

            """ or """
            self.mask = cv2.inRange(self.hsv, self.lower[self.key], self.upper[self.key])
            self.mask = cv2.erode(self.mask, None, iterations=2)
            self.mask = cv2.dilate(self.mask, None, iterations=2)

            """ find contours in the mask and initialize the current (x, y) center of the ball """
            self.cnts = cv2.findContours(self.mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            self.center = None

            """ only proceed if at least one contour was found """
            if len(self.cnts) > 0:
                """ find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid """
                self.c = max(self.cnts, key=cv2.contourArea)
                ((self.x, self.y), self.radius) = cv2.minEnclosingCircle(self.c)

                """ Image moments help you to calculate some features like center of mass of the object, area of the object etc """
                """ http://en.wikipedia.org/wiki/Image_moment """
                self.M = cv2.moments(self.c)

                """ Centroid is given by the relations, Cx=M10/M00 and Cy=M01/M00 """
                self.center = (int(self.M["m10"] / self.M["m00"]), int(self.M["m01"] / self.M["m00"]))

                # TODO: Find a good way to publish multiple ROI
                # --------------------------------------------------------------
                """ Straight Bounding Rectangle """
                # self.xBox, self.yBox, self.w, self.h = cv2.boundingRect(self.c)

                """ RegionOfInterest """
                # roi = RegionOfInterest()
                # roi.x_offset = self.xBox
                # roi.y_offset = self.yBox
                # roi.width = self.w
                # roi.height = self.h
                #
                # self.imgROI_pub.publish(roi)

                """ Compute the center of the ROI """
                # COG_x = roi.x_offset + roi.width / 2 - self.image_width / 2
                # COG_y = roi.y_offset + roi.height / 2 - self.image_height / 2
                # --------------------------------------------------------------

                """ only proceed if the radius meets a minimum size. Correct this value for your obect's size """
                if self.radius > 10:
                    """ draw the circle and centroid on the frame, then update the list of tracked points """
                    cv2.circle(self.cv_image, (int(self.x), int(self.y)), int(self.radius), self.colors[self.key], 2)
                    cv2.circle(self.cv_image, (int(self.x), int(self.y)), 5, self.colors[self.key], -1) # center points
                    cv2.putText(self.cv_image, self.key + " ball", (int(self.x - self.radius),int(self.y - self.radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.colors[self.key],2)

def usage():
    print("Please specify topic name:")
    print("%s [Topic Name]" % sys.argv[0])

    # if len(sys.argv) == 3:
    #     print("Multiple Ball Tracking Node [ONLINE]")
    #     print("[Topic Name]: %s" % sys.argv[1])
    #     print("[Topic Name]: %s" % sys.argv[2])
    #     main(sys.argv)
    # else:
    #     print(usage())
    #     sys.exit(1)

def main(args):
    vn = multiple_color_tracking_node()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

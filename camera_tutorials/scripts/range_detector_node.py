#!/usr/bin/env python

from __future__ import print_function
import roslib
roslib.load_manifest('camera_tutorials')
import sys
import rospy
import os
import cv2
import imutils
from std_msgs.msg import String
from camera_tutorials.msg import IntList
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class range_detector_node:
    def __init__(self, filter):

        self.filter = filter

        """ Initializing your ROS Node """
        """ rospy.init_node('my_node_name', anonymous=True) ==> unique name """
        """ or """
        """ rospy.init_node('my_node_name') """
        rospy.init_node('range_detector_node', anonymous=True)

        """ Give the OpenCV display window a name """
        self.cv_window_original = "Original"
        self.cv_window_thresh = "Thresh"
        self.cv_window_preview = "Preview"

        # self.range_pub = rospy.Publisher(self.pub_name, IntList, queue_size=10)

        """ Create the cv_bridge object """
        self.bridge = CvBridge()

        """ Subscribe to the raw camera image topic """
        """# subscribe to a topic using rospy.Subscriber class """
        """ sub=rospy.Subscriber('TOPIC_NAME', TOPIC_MESSAGE_TYPE, name_callback) """
        self.image_sub = rospy.Subscriber("/cam0/image_raw", Image, self.callback)

    def callback(self, data):
        """ Convert the raw image to OpenCV format using the convert_image() helper function """
        self.convert_image(data)

        """ Determine the range_filter using the setup_trackbars() helper function """
        self.setup_trackbars()

        if self.filter.upper() == 'RGB':
            self.frame_to_thresh = self.cv_image.copy()
        else:
            self.frame_to_thresh = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

        """ self.get_trackbar_values() """
        v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = self.get_trackbar_values()

        """ publish """
        # msg = IntList()
        # msg.v1_min = v1_min
        # msg.v2_min = v2_min
        # msg.v3_min = v3_min
        # msg.v1_max = v1_max
        # msg.v2_max = v2_max
        # msg.v3_max = v3_max
        # self.range_pub.publish(msg)

        self.thresh = cv2.inRange(self.frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

        """ optional (un-comment to preview) """
# ------------------------------------------------------------------------------
        """ Get the width(cols), height(rows), and channels  of the image """
        # (rows,cols,channels) = self.cv_image.shape

        """ optional (un-comment to preview) """
        # if cols > 60 and rows > 60:
        #     cv2.circle(self.cv_image, (50, 50), 10, 255, -1)

        """ Overlay some text onto the image display """
        # fontface = cv2.FONT_HERSHEY_DUPLEX
        # fontscale = 2
        # fontcolor = (255, 255, 255)
        # cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]])
        # cv2.putText(self.cv_image, self.cv_window_original, (50, rows / 2), fontface, fontscale, fontcolor, 1)
# ------------------------------------------------------------------------------

        """ displaying an OpenCV image """
        self.preview = cv2.bitwise_and(self.cv_image, self.cv_image, mask=self.thresh)

        # cv2.imshow(self.cv_window_original, self.cv_image)
        # cv2.imshow(self.cv_window_thresh, self.thresh)
        cv2.imshow(self.cv_window_preview, self.preview)

        cv2.waitKey(1)

# ------------------------------------------------------------------------------

    def convert_image(self, ros_image):
        """ coverting the ROS image data to uint8 OpenCV format """
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")

            """ optional (un-comment to reshape the original image) """
# ------------------------------------------------------------------------------
            # self.cv_image = imutils.resize(self.cv_image, width=320)

            """ optional (un-comment to flip the original image) """
# ------------------------------------------------------------------------------
            self.cv_image = cv2.flip(self.cv_image, 1)

        except CvBridgeError as e:
            print(e)

    def setup_trackbars(self):
        self.cv_window_trackbar = "Trackbars"
        cv2.namedWindow(self.cv_window_trackbar, 0)

        for i in ["MIN", "MAX"]:
            v = 0 if i == "MIN" else 255

            for j in self.filter.upper():
                cv2.createTrackbar("%s_%s" % (j, i), self.cv_window_trackbar, v, 255, self.callback_trackbars)

    def callback_trackbars(self, value):
        pass

    def get_trackbar_values(self):
        values = []

        for i in ["MIN", "MAX"]:
            for j in self.filter.upper():
                v = cv2.getTrackbarPos("%s_%s" % (j, i), self.cv_window_trackbar)
                values.append(v)

        return values

def usage():
    print("Please specify type of color filter:")
    print("1. RGB")
    print("2. HSV")
    print("%s [tracker]" % sys.argv[0])

def main(args):
    vn = range_detector_node(sys.argv[1])

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    if len(sys.argv) == 2:
        main(sys.argv)
    else:
        print(usage())
        sys.exit(1)

#!/usr/bin/env python

from __future__ import print_function
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class test_vision_node:
    def __init__(self):
        # Initializing your ROS Node
        # rospy.init_node('my_node_name', anonymous=True)
        # or
        # rospy.init_node('my_node_name')
        rospy.init_node('test_vision_node')

        # Give the OpenCV display window a name
        self.cv_window_name = "OpenCV Image"

        # Create the window and make it re-sizeable (second parameter = 0)
        # cv2.namedWindow(self.cv_window_name, cv2.WINDOW_NORMAL)

        # rospy.Publisher initialization
        # pub = rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10)
        # The only required arguments to create a rospy.Publisher are the topic name, the Message class, and the queue_size
        self.image_pub = rospy.Publisher("/opencv_img", Image, queue_size=10)
        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Subscribe to the raw camera image topic
        # subscribe to a topic using rospy.Subscriber class
        # sub=rospy.Subscriber('TOPIC_NAME', TOPIC_MESSAGE_TYPE, name_callback)
        self.image_sub = rospy.Subscriber("/cv_camera_node/cam0/image_raw", Image, self.callback)

    def callback(self,data):
        try:
            # coverting the ROS image data to uint8 OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:

            print(e)
# ------------------------------------------------------------------------------
# Un-comment this area to view image
        # Get the width(cols), height(rows), and channels  of the image
        (rows,cols,channels) = cv_image.shape

        # optional (un-comment to preview)
        if cols > 60 and rows > 60:
            cv2.circle(cv_image, (50, 50), 10, 255, -1)

        # Overlay some text onto the image display
        fontface = cv2.FONT_HERSHEY_DUPLEX
        fontscale = 2
        fontcolor = (255, 255, 255)
        # cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]])
        cv2.putText(cv_image, self.cv_window_name, (50, rows / 2), fontface, fontscale, fontcolor, 1)

        # displaying an OpenCV image
        cv2.imshow(self.cv_window_name, cv_image)
        cv2.waitKey(1)
# ------------------------------------------------------------------------------

        try:
            # coverting the uint8 OpenCV image to ROS image data
            # Publisher.publish() -- explicit way
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    vn = test_vision_node()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

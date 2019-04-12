#!/usr/bin/env python

from __future__ import print_function
import roslib
roslib.load_manifest('camera_tutorials')
import sys
import rospy
import cv2
import imutils
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class test_vision_node:
    def __init__(self):

        """  Initializing your ROS Node """
        """  rospy.init_node('my_node_name', anonymous=True) """
        """  or """
        """  rospy.init_node('my_node_name') """
        rospy.init_node('test_vision_node', anonymous=True)

        """ Give the OpenCV display window a name """
        self.cv_window_name = "OpenCV Image"

        """ rospy.Publisher initialization """
        """ pub = rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10) """
        """ The only required arguments to create a rospy.Publisher are the topic name, the Message class, and the queue_size """

        """ Publish as the opencv image topic """
        self.image_pub = rospy.Publisher("/opencv_img", Image, queue_size=10)

        """ Create the cv_bridge object """
        self.bridge = CvBridge()

        """ subscribe to a topic using rospy.Subscriber class """
        """ sub = rospy.Subscriber('TOPIC_NAME', TOPIC_MESSAGE_TYPE, name_callback) """

        """ Subscribe to the raw camera image topic """
        self.image_sub = rospy.Subscriber("/cam0/image_raw", Image, self.callback)

        """  Subscribe to the info camera topic """
        self.imgInfo_sub = rospy.Subscriber("/cam0/camera_info", CameraInfo, self.getCameraInfo)

    def callback(self,data):
        """ Convert the raw image to OpenCV format using the convert_image() helper function """
        self.cvtImage(data)

        """ Do some image processing; flip, resize, and etc"""
        self.imgProcessing()

        """ displaying an OpenCV image """
        cv2.imshow(self.cv_window_name, self.cv_image)
        cv2.waitKey(1)
# ------------------------------------------------------------------------------

        try:
            """ coverting the uint8 OpenCV image to ROS image data """
            """ Publisher.publish() -- explicit way """
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def cvtImage(self, ros_image):
        """ coverting the ROS image data to uint8 OpenCV format """
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            # self.cv_image_copy = self.cv_image.copy()

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

def usage():
    # sys.argv[0] ==> name of the script
    print("Please specify rostopic to subscribe/publish:")
    print("%s [subscribed/published topic name]" % sys.argv[0])

    if len(sys.argv) == 3:
        print("[Subscribed Topic Name]: %s " % sys.argv[1])
        print("[Published Topic Name]: %s " % sys.argv[2])
        main(sys.argv)
    else:
        print(usage())
        sys.exit(1)

def main(args):
    vn = test_vision_node()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

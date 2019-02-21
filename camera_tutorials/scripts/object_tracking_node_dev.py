#!/usr/bin/env python

from __future__ import print_function
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
import imutils
from imutils.video import FPS
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

class object_tracking_node:
    def __init__(self, type_tracker):

        self.type_tracker = type_tracker

        # Initializing your ROS Node
        # rospy.init_node('my_node_name', anonymous=True)
        # or
        # rospy.init_node('my_node_name')
        rospy.init_node('object_tracking_node')

        # Give the OpenCV display window a name
        self.cv_window_name_0 = "OpenCV Image"
        self.cv_window_name_1 = "OpenCV Image Mask"

        # initialize a dictionary that maps strings to their corresponding OpenCV object tracker implementations
        OPENCV_OBJECT_TRACKERS = {
        # "csrt": cv2.TrackerCSRT_create,
        "kcf": cv2.TrackerKCF_create,
        "boosting": cv2.TrackerBoosting_create,
        "mil": cv2.TrackerMIL_create,
        "tld": cv2.TrackerTLD_create,
        "medianflow": cv2.TrackerMedianFlow_create,
        "mosse": cv2.TrackerMOSSE_create
        }

        # initialize OpenCV's special multi-object tracker
        self.trackers = cv2.MultiTracker_create()

        # initialize the bounding box coordinates of the object we are going to track
        self.initBB = None

        # initialize the FPS throughput estimator
        self.fps = None

        # rospy.Publisher initialization
        # pub = rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10)
        # The only required arguments to create a rospy.Publisher are the topic name, the Message class, and the queue_size
        self.roi_pub = rospy.Publisher("roi", RegionOfInterest, queue_size=10)

        # Give the camera driver a moment to come up
        rospy.sleep(1)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Subscribe to the raw camera image topic
        # subscribe to a topic using rospy.Subscriber class
        # sub=rospy.Subscriber('TOPIC_NAME', TOPIC_MESSAGE_TYPE, name_callback)
        self.image_sub = rospy.Subscriber("/camera0/cam0/image_raw", Image, self.callback)

    def callback(self, data):
        # Convert the raw image to OpenCV format using the convert_image() helper function
        self.convert_image(data)

        # Apply image processing using do_preprocess() helper function
        self.do_preprocess()

        # Apply tracking using do_track() helper function
        self.do_track()

        # self.do_trackPts()

        # Refresh the displayed image
        cv2.imshow(self.cv_window_name_0,np.hstack([self.cv_image]))

    def convert_image(self, ros_image):
        # coverting the ROS image data to uint8 OpenCV format
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print(e)

    def do_preprocess(self):
        try:
            # resize the frame (so we can process it faster) and grab the frame dimensions
            self.cv_image = imutils.resize(self.cv_image, width=320)
            (self.H, self.W) = self.cv_image.shape[:2]

        except CvBridgeError as e:
            print (e)

    def do_track(self):
        try:
            # grab the updated bounding box coordinates (if any) for each object that is being tracked
            (success, boxes) = self.trackers.update(self.cv_image)

            # loop over the bounding boxes and draw then on the frame
            for box in boxes:
                (x, y, w, h) = [int(v) for v in box]
                cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            self.key = cv2.waitKey(1) % 0xFF

            # Toggle between the normal and target selection "s"
            if self.key == ord("q"):
                # initialize a dictionary that maps strings to their corresponding OpenCV object tracker implementations
                OPENCV_OBJECT_TRACKERS = {
                # "csrt": cv2.TrackerCSRT_create,
                "kcf": cv2.TrackerKCF_create,
                "boosting": cv2.TrackerBoosting_create,
                "mil": cv2.TrackerMIL_create,
                "tld": cv2.TrackerTLD_create,
                "medianflow": cv2.TrackerMedianFlow_create,
                "mosse": cv2.TrackerMOSSE_create
                }

                # initialize OpenCV's special multi-object tracker
                self.trackers = cv2.MultiTracker_create()

            # if the 's' key is selected, we are going to "select" a bounding box to track
            elif self.key == ord("s"):
                # select the bounding box of the object we want to track (make sure you press ENTER or SPACE after selecting the ROI)
                self.box = cv2.selectROI(self.cv_window_name_0, self.cv_image, fromCenter=False, showCrosshair=True)

                OPENCV_OBJECT_TRACKERS = {
                # "csrt": cv2.TrackerCSRT_create,
                "kcf": cv2.TrackerKCF_create,
                "boosting": cv2.TrackerBoosting_create,
                "mil": cv2.TrackerMIL_create,
                "tld": cv2.TrackerTLD_create,
                "medianflow": cv2.TrackerMedianFlow_create,
                "mosse": cv2.TrackerMOSSE_create
                }

                # create a new object tracker for the bounding box and add it to our multi-object tracker
                self.tracker = OPENCV_OBJECT_TRACKERS[self.type_tracker]()
                self.trackers.add(self.tracker, self.cv_image, self.box)

        except CvBridgeError as e:
            print (e)

    def do_select(self):
        pass


def usage():
    print("Please specify type of tracker:")
    # print("1. csrt")
    print("2. kcf")
    print("3. boosting")
    print("4. mil")
    print("5. tld")
    print("6. medianflow")
    print("7. mosse")
    print("%s [tracker]" % sys.argv[0])

def main(args):
    vn = object_tracking_node(sys.argv[1])
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

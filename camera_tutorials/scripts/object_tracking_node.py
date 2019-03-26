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

        # extract the OpenCV version info
        (major, minor) = cv2.__version__.split(".")[:2]

        # if we are using OpenCV 3.2 OR BEFORE, we can use a special factory function to create our object tracker
        if int(major) == 3 and int(minor) < 3:
            self.tracker = cv2.Tracker_create([self.type_tracker].upper())

        # otherwise, for OpenCV 3.3 OR NEWER, we need to explicity call the
        # approrpiate object tracker constructor:
        else:
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

            # grab the appropriate object tracker using our dictionary of OpenCV object tracker objects
            self.tracker = OPENCV_OBJECT_TRACKERS[self.type_tracker]()

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
        self.image_sub = rospy.Subscriber("/cv_camera_node/cam0/image_raw", Image, self.callback)
        self.image_info = rospy.Subscriber('/cv_camera_node/cam0/image_raw/camera_info', CameraInfo, self.do_preprocess)

    def callback(self, data):
        # Convert the raw image to OpenCV format using the convert_image() helper function
        self.convert_image(data)

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

    def do_preprocess(self, data):
        self.W = data.width
        self.H = data.height

    def do_track(self):
        try:
            # check to see if we are currently tracking an object
            if self.initBB is not None:
                # grab the new bounding box coordinates of the object
                (success, box) = self.tracker.update(self.cv_image)

                # check to see if the tracking was a success
                if success:
                    (x, y, w, h) = [int(v) for v in box]
                    cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    # RegionOfInterest
                    roi = RegionOfInterest()
                    roi.x_offset = x
                    roi.y_offset = y
                    roi.width = w
                    roi.height = h

                    self.roi_pub.publish(roi)

                # update the FPS counter
                self.fps.update()
                self.fps.stop()

                # initialize the set of information we'll be displaying on the frame
                info = [
                ("Tracker", self.type_tracker),
                ("Success", "Yes" if success else "No"),
                ("FPS", "{:.2f}".format(self.fps.fps())),
                ]

                # loop over the info tuples and draw them on our frame
                for (i, (k, v)) in enumerate(info):
                    text = "{}: {}".format(k, v)
                    cv2.putText(self.cv_image, text, (10, self.H - ((i * 20) + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            self.key = cv2.waitKey(1) % 0xFF

            # Toggle between the normal and target selection "s"
            if self.key == ord("q"):
                # self.tracker.release()
                OPENCV_OBJECT_TRACKERS = {
                # "csrt": cv2.TrackerCSRT_create,
                "kcf": cv2.TrackerKCF_create,
                "boosting": cv2.TrackerBoosting_create,
                "mil": cv2.TrackerMIL_create,
                "tld": cv2.TrackerTLD_create,
                "medianflow": cv2.TrackerMedianFlow_create,
                "mosse": cv2.TrackerMOSSE_create
                }

                self.tracker = OPENCV_OBJECT_TRACKERS[self.type_tracker]()
                # self.initBB = None
                # self.tracker.init(self.cv_image, self.initBB)
                # self.fps = None
                # box = []

            # if the 's' key is selected, we are going to "select" a bounding box to track
            elif self.key == ord("s"):
                # select the bounding box of the object we want to track (make sure you press ENTER or SPACE after selecting the ROI)
                self.initBB = cv2.selectROI(self.cv_window_name_0, self.cv_image, fromCenter=False, showCrosshair=True)

                # start OpenCV object tracker using the supplied bounding box coordinates, then start the FPS throughput estimator as well
                self.tracker.init(self.cv_image, self.initBB)
                self.fps = FPS().start()

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

#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Shot():
    def __init__(self):
        self.image_sub = rospy.Subscriber("/hsrb/head_center_camera/image_raw", Image, self.callback)
        self.bridge = CvBridge()

        
    def callback(self, data):
        try:
            self.cv_image = self.brindge.imgmsg_to_cv2(data, "bgr8")
            self.original_image = copy.copy(self.cv_image)

        except CvBridgeError as e:
            print("Cv_Bridge_Error")

        gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

        half_image = cv2.resize(self.original_image, (0,0), fx=0.5, fy=0.5)
        cv2.imshow("Origin Image", half_image)
        cv2.imshow("Result Image", gray_image)
        cv2.waitKey(3)

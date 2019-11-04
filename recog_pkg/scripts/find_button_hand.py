#!/usr/bin/env python

import rospy
import hsrb_interface
from hsrb_interface import geometry
from hsrb_interface import exceptions
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Quaternion
#from recog_pkg.msg import Camera3d
import tf2_ros
import tf_conversions
import math
import numpy as np
from std_msgs.msg import Float32, Int32
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from jsk_recognition_msgs.msg import LabelArray, RectArray
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
import cv_bridge
from recog_pkg.msg import Camera3d, HandCamera2d
import copy
import tmc_eus_py
from tmc_eus_py.coordinates import Coordinates

class FindButtonHand():
    def __init__(self):
        #def get_3d_point_hand(self):
        #rospy.init_node("two2three_hand")
        #self.pub_viz = rospy.Publisher("/hand_camera_remote/image_raw", Image, queue_size=1)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("~output", HandCamera2d, queue_size=2**24)
        self.sub = message_filters.Subscriber("/hand_camera_remote/image_raw", Image, queue_size=1)
        #self.sub_labelArray = message_filters.Subscriber("/qatm/output/labels", LabelArray, queue_size=1)
        self.sub_rectArray = message_filters.Subscriber("/qatm2/output/rects", RectArray, queue_size=1)
        queue_size = rospy.get_param("~queue_size", 100)
        self.subs = [self.sub, self.sub_rectArray]
        slop = rospy.get_param("~slop", 0.1)
        self.sync = message_filters.ApproximateTimeSynchronizer(self.subs, queue_size=queue_size, slop=slop)
        self.sync.registerCallback(self.hand_camera_cb)

    # def unsubscribe(self):
    #     for sub in self.subs:
    #         sub.unregister()

    def hand_camera_cb(self, msg1, rectmsg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg1, "passthrough")
        except CvBridgeError, e:
            rospy.logerr(e)

        self.h, self.w, _ = depth_image.shape

        rect_info = rectmsg.rects[0]
        self.u = rect_info.x + rect_info.width/2
        self.v = rect_info.y + rect_info.height/2
        print(self.u, self.v)
        msg = HandCamera2d(x=self.u, y=self.v)
        self.pub.publish(msg)
        #output_image = self.plot_ok(depth_image)
        #msg_viz = cv_bridge.CvBridge().cv2_to_imgmsg(output_image, encoding="bgr8")
        #self.pub_viz.publish(msg_viz)

    # def plot_ok(self,image_raw, show=False, save_name=None):
    #     d_img = image_raw.copy()
    #     d_img = cv2.putText(d_img, "OK", (self.u_low, self.v_low), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,0,0))
    #     d_img = cv2.rectangle(d_img, (self.u_low, self.v_low), (self.u_high, self.v_high), (255,0,0))
    #     if show:
    #         plt.imshow(d_img)
    #     if save_name:
    #         cv2.imwrite(save_name, d_img[:,:,::-1])
    #     return d_img

if __name__ == "__main__":
    rospy.init_node("find_button_hand")
    FindButtonHand()
    rospy.spin()

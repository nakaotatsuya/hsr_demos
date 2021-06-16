#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image, CameraInfo
from jsk_recognition_msgs.msg import LabelArray, RectArray
import math
import message_filters
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
from recog_pkg.msg import Camera3d
#from image_geometry import PinholeCameraModel

class Two2Three():
    def __init__(self, camera):
        #rospy.init_node("two2three")
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("~output", Camera3d, queue_size=2**24)
        if camera == "head":
            self.get_3d_point()
        # elif camera == "hand":
        #     self.get_3d_point_hand()

    def get_3d_point(self):
        self.sub_depth = message_filters.Subscriber("/head_rgbd_sensor_remote/depth_registered/image", Image, queue_size=1)
        # self.sub_labelArray = message_filters.Subscriber("/qatm/output/labels", LabelArray, queue_size=1)
        self.sub_rectArray = message_filters.Subscriber("/qatm/output/rects", RectArray, queue_size=1)
        self.sub_camera_info = message_filters.Subscriber("/head_rgbd_sensor_remote/depth_registered/camera_info", CameraInfo, queue_size=1)
        queue_size = rospy.get_param("~queue_size", 100)
        self.subs = [self.sub_depth, self.sub_rectArray, self.sub_camera_info]
        #if rospy.get_param("~approximate_sync", False):
        slop = rospy.get_param("~slop", 0.1)
        self.sync = message_filters.ApproximateTimeSynchronizer(self.subs, queue_size=queue_size, slop=slop)
        #else:
        #    self.sync = message_filters.TimeSynchronizer(
        #        self.subs, queue_size=queue_size)
        self.sync.registerCallback(self.cb)

    # def get_3d_point_hand(self):
    #     self.sub_depth = message_filters.Subscriber("/hsrb/hand_camera/image_raw", Image, queue_size=1)
    #     #self.sub_labelArray = message_filters.Subscriber("/qatm/output/labels", LabelArray, queue_size=1)
    #     self.sub_rectArray = message_filters.Subscriber("/qatm2/output/rects", RectArray, queue_size=1)
    #     self.sub_camera_info = message_filters.Subscriber("/hsrb/hand_camera/camera_info", CameraInfo, queue_size=1)
    #     queue_size = rospy.get_param("~queue_size", 100)
    #     self.subs = [self.sub_depth, self.sub_rectArray, self.sub_camera_info]
    #     #if rospy.get_param("~approximate_sync", False):
    #     slop = rospy.get_param("~slop", 0.1)
    #     self.sync = message_filters.ApproximateTimeSynchronizer(self.subs, queue_size=queue_size, slop=slop)
    #     #else:
    #     #    self.sync = message_filters.TimeSynchronizer(
    #     #        self.subs, queue_size=queue_size)
    #     self.sync.registerCallback(self.cb)


        #rospy.Subscriber("/head_rgbd_sensor_remote/depth_registered/image", Image, self.cb)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def cb(self, msg1, rectmsg, cameramsg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg1, "passthrough")
        except CvBridgeError, e:
            rospy.logerr(e)

        h, w = depth_image.shape #h=480,w=640

        cx = cameramsg.K[2]
        cy = cameramsg.K[5]
        fx = cameramsg.K[0]
        fy = cameramsg.K[4]

        #print("labelmsg = ")
        #print(labelmsg.labels[1].id)
        #for i in range(len(labelmsg.labels)):
        #    if labelmsg.labels[i].name=="faucet":
        #        faucet_id = labelmsg.labels[i].id
        #        print(faucet_id)
        rect_info = rectmsg.rects[0]
        u = rect_info.x
        v = rect_info.y
        x1 = u
        x2 = u + rect_info.width
        y1 = v
        y2 = v + rect_info.height
        sum = 0
        area = 0
        for i in range(y1, y2):
            for j in range(x1, x2):
                if depth_image.item(i,j) == depth_image.item(i,j):
                    sum += depth_image.item(i,j)
                    area += 1

        try:
            self.camera_z = sum / area
        except ZeroDivisionError:
            print("cannot divide zero")
            self.camera_z= 10
        #print(camera_z)
        #print(area)

        self.camera_x = self.camera_z * (u + rect_info.width/2 - cx) / fx
        self.camera_y = self.camera_z * (v + rect_info.height/2 - cy) / fy
        #print(rectmsg.rects[0])
        #print(self.camera_x, self.camera_y, self.camera_z)

        self.publish()

    def publish(self):
        msg = Camera3d(x=self.camera_x, y=self.camera_y, z=self.camera_z)
        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("two2three")
    Two2Three("head")
    #two2three.get_3d_point()
    rospy.spin()

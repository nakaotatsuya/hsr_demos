#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import math
import sys
import actionlib

import rospy
import rospkg
import numpy as np
import message_filters
import sympy
import tf
import tf.transformations
from os import makedirs, listdir
from os import path as osp

from tmc_eus_py.coordinates import Coordinates
from visualization_msgs.msg import Marker
from hark_msgs.msg import HarkSource
from sound_classification.msg import InSound
from geometry_msgs.msg import Point, PointStamped, Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA

import argparse
import line_notify

import hsrb_interface
from hsrb_interface import geometry
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

from jsk_recognition_msgs.msg import LabelArray, BoundingBoxArray, ClassificationResult, PeoplePoseArray
from hark_msgs.msg import HarkSource, HarkWave
from jsk_hsr_startup.robot_action import RobotAction
from jsk_hsr_startup.spot_mixin import SpotMixin
from jsk_hsr_startup.tmc_speak import speak_jp
from geometry_msgs.msg import PolygonStamped, PointStamped
from std_msgs.msg import Header
from my_robot import MyRobot

from sensor_msgs.msg import Image
from PIL import Image as _Image
from cv_bridge import CvBridge

class RecognizeSoundPos(MyRobot):
    def __init__(self, name):
        super(RecognizeSoundPos, self).__init__(name)
        self.whole_body = self.robot.get("whole_body")
        #self.omni_base = self.robot.get("omni_base")
        rospack = rospkg.RosPack()
        self.client = line_notify.LineNotify(token="PuanmDsKMVCqqXdhpkh7MFth9qxsbhfRPTuBpZHGUvH")
        self.client.notify("init")

        self.flag = False
        self.ins_buffer = np.array([])
        self.time = Header()
        self.time2 = Header()
        self.threshold = rospy.get_param("~threshold", 1.2)
        self.bridge = CvBridge()
        self.sound_class_label = np.array([])
        self.save_dir = osp.join(rospack.get_path(
            "action_pkg"), "img_data")

    ####
    def subscribe_in_sound(self):
        self.in_sound = rospy.Subscriber("/tamago1/harkwave", HarkWave, self.callback_in_sound, queue_size=1, buff_size=2**24)

    def unsubscribe_in_sound(self):
        self.in_sound.unregister()

    def callback_in_sound(self, ins_msg):
        for i in range(len(ins_msg.src)):
            test = np.array(ins_msg.src[i].wavedata)
            #print(test.max())
            self.ins_buffer = np.append(self.ins_buffer, test.max())
        self.ins_buffer = self.ins_buffer[-400:]

        if np.all(self.ins_buffer[0:8] >= self.threshold):
            self.time = ins_msg.header
            self.flag = True
    ####
    
    ####
    def subscribe(self):
        sound_direction = message_filters.Subscriber("/wavdata_node_copy/max", HarkSource, queue_size=1)
        sound_class = message_filters.Subscriber("/sound_classifier/output", ClassificationResult, queue_size=1)
        use_async = rospy.get_param("~approximate_sync", True)
        queue_size = rospy.get_param("~queue_size", 1)
        self.subs = [sound_direction, sound_class]
        if use_async:
            slop = rospy.get_param("~slop", 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                self.subs, queue_size, slop)
        else:
            sync = message_filters.TimeSynchronizer(self.subs, queue_size)
        sync.registerCallback(self.callback)
        
        #self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)
    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def callback(self, sd_msg, sc_msg):
        self.time2 = sd_msg.header
        print(self.time2.stamp.to_sec())

        max_direction = sd_msg.src[0]
        max_point, max_azimuth, max_elevation = self.dir_to_point(max_direction)
        print("max_point:", max_point)
        self.max_point = geometry.vector3(max_point[0], max_point[1], max_point[2])

        #map_point = PointStamped()
        #map_point.header.frame_id = "map"

        self.sound_class_label = np.append(self.sound_class_label ,sc_msg.label_names[0])
        self.sound_class_label = self.sound_class_label[-5:]
    ####

    def gaze_point(self):
        max_point_list = []
        self.subscribe()
        #self.subscribe_in_sound()
        self.subscribe_camera()
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while not self.flag:
            rate.sleep()
            print(self.sound_class_label)
            if all([e == "key\n" for e in self.sound_class_label]) and len(self.sound_class_label)==5:
                rospy.sleep(1.0)
                max_point_list.append(self.max_point)
                self.flag = True
            if (rospy.Time.now() - start_time).to_sec() > 30.0:
                return True
        #self.unsubscribe_in_sound()

        print("time:", self.time.stamp.to_sec())
        print("now:", rospy.Time.now().to_sec())

        # while self.time.stamp.to_sec() + 1.0 > rospy.Time.now().to_sec():
        #     rate.sleep()
        #     max_point_list.append(self.max_point)
        #     #direction_vector_list.append(self.direction_vector)
        #     if (rospy.Time.now() - start_time).to_sec() > 20.0:
        #         return True
        self.unsubscribe()

        print(max_point_list)
        self.client.notify("{} is ringing.".format(self.sound_class_label[0]))
        print("move around")

        if max_point_list[-1].z > 0.1:
            #self.whole_body.move_to_joint_positions({"arm_lift_joint": 0.4})
            self.whole_body.move_to_joint_positions({"arm_lift_joint": 0.4, "arm_flex_joint": -0.7})
        self.whole_body.gaze_point(point=max_point_list[-1], ref_frame_id="tamago1")

        rospy.sleep(1.0)
        img_file = osp.join(self.save_dir, "{}_0000.jpg".format(self.sound_class_label[0]))
        cv2.imwrite(img_file, cv2.cvtColor(self.color.astype(np.uint8), cv2.COLOR_BGR2RGB))
        print("save camera")
        
        pil_img = _Image.open(img_file)
        self.client.notify(imgs=pil_img)
    ####

    ####
    def subscribe_camera(self):
        sub_camera = rospy.Subscriber("/remote/head_rgbd_sensor/rgb/image_rect_color", Image, self.callback_camera, queue_size=1)

    def callback_camera(self, img_msg):
        self.color = self.bridge.imgmsg_to_cv2(img_msg, "rgb8")

if __name__=="__main__":
    rospy.init_node("recognize_sound_pos")
    recognize_sound_pos = RecognizeSoundPos("a")
    recognize_sound_pos.gaze_point()
    

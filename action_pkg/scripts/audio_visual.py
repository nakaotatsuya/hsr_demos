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

from jsk_recognition_msgs.msg import LabelArray, BoundingBoxArray, ClassificationResult, PeoplePoseArray, Accuracy
from hark_msgs.msg import HarkSource, HarkWave
from jsk_hsr_startup.robot_action import RobotAction
from jsk_hsr_startup.spot_mixin import SpotMixin
from jsk_hsr_startup.tmc_speak import speak_jp
from geometry_msgs.msg import PolygonStamped, PointStamped, PoseArray
from std_msgs.msg import Header
from my_robot import MyRobot

from sensor_msgs.msg import Image
from PIL import Image as _Image
from cv_bridge import CvBridge

class AudioVisual(MyRobot):
    def __init__(self, name):
        super(AudioVisual, self).__init__(name)
        self.use_async = True
        self.target_boxes = []

    #subscribe box
    def subscribe_box(self):
        label_array = message_filters.Subscriber(
            "/mask_rcnn_instance_segmentation/output/labels", LabelArray)
        bbox_array = message_filters.Subscriber(
            "/multi_euclidean_cluster_point_indices_decomposer/boxes", BoundingBoxArray)
        pose_array = message_filters.Subscriber(
            "/multi_euclidean_cluster_point_indices_decomposer/centroid_pose_array", PoseArray)
        self.subs_box = [label_array, bbox_array, pose_array]
        if self.use_async:
            sync = message_filters.ApproximateTimeSynchronizer(self.subs_box, queue_size=1, slop=0.1)
        else:
            sync = message_filters.TimeSynchronizer(self.subs_box, queue_size=1)
        sync.registerCallback(self.box_callback)

    def unsubscribe_box(self):
        for sub in self.subs_box:
            sub.unregister()

    def box_callback(self, labels, boxes, poses):
        target_boxes = []
        #grasp_hand_size = 0.08
        for label, box, pose in zip(labels.labels, boxes.boxes, poses.poses):
            print(label)
            #if box.dimensions.y < grasp_hand_size:
            target_boxes.append((label, box, pose))
        self.target_boxes = target_boxes
        print("num:", len(self.target_boxes))

    def test(self):
        c = self.get_a_to_b("map", "base_link")

        self.subscribe_box()
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        self.target_boxes = []
        while len(self.target_boxes) == 0:
            rate.sleep()
            if (rospy.Time.now() - start_time).to_sec() > 10.0:
                return True
        self.unsubscribe_box()

        target_box = self.target_boxes[0][1]
        pos = np.array([target_box.pose.position.x, target_box.pose.position.y, target_box.pose.position.z])
        #print(target_box)
        pos_from_map = c.transform_vector(pos)
        print(pos_from_map)

if __name__=="__main__":
    rospy.init_node("audio_visual")
    audio_visual = AudioVisual("a")
    audio_visual.test()


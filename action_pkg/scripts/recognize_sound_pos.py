#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
from geometry_msgs.msg import Point, PointStamped, Point, Quaternion
from std_msgs.msg import ColorRGBA

import argparse
import line_notify

import hsrb_interface
from hsrb_interface import geometry
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

from jsk_recognition_msgs.msg import LabelArray, BoundingBoxArray, ClassificationResult, PeoplePoseArray
from hark_msgs.msg import HarkSource
from jsk_hsr_startup.robot_action import RobotAction
from jsk_hsr_startup.spot_mixin import SpotMixin
from jsk_hsr_startup.tmc_speak import speak_jp
from geometry_msgs.msg import PolygonStamped, PointStamped

class RecognizeSoundPos():
    def __init__(self):
        rospack = rospkg.RosPack()
        client = line_notify.LineNotify(token="PuanmDsKMVCqqXdhpkh7MFth9qxsbhfRPTuBpZHGUvH")
        client.notify("init")

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
        sync.registerCallback(self._callback)
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()
        


#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import numpy as np
import message_filters
import sympy
import tf
import tf.transformations
from os import makedirs, listdir
from os import path as osp

#from tmc_eus_py.coordinates import Coordinates
from visualization_msgs.msg import Marker
from hark_msgs.msg import HarkSource, HarkWave
from sound_classification.msg import InSound
from geometry_msgs.msg import Point, PointStamped, Point, Quaternion
from std_msgs.msg import Header

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

from jsk_hsr_startup.robot_action import RobotAction
from jsk_hsr_startup.spot_mixin import SpotMixin
from jsk_hsr_startup.tmc_speak import speak_jp
from my_robot import MyRobot
import argparse

from std_msgs.msg import ColorRGBA

#visualize max sound direction. Considering the gauss sound map, robots can estimate the precise position of where the sound happens.

class VisualizeSoundMax(MyRobot):
    def __init__(self, name):
        super(VisualizeSoundMax, self).__init__(name)

        self.rospack = rospkg.RosPack()
        # parser = argparse.ArgumentParser()
        # parser.add_argument("-l", "--label", type=str, default="test")
        # parser.add_argument("-th", "--threshold", type=float, default="1.0")
        # args = parser.parse_args()

        self.label = rospy.get_param("~label", "key")
        # self.save_dir = osp.join(rospack.get_path(
        #     'action_pkg'), "sound_pos_data", self.label)
        # if not osp.exists(self.save_dir):
        #     makedirs(self.save_dir)
        self.listener = tf.TransformListener()

        self.flag = False
        self.ins_buffer = np.array([])
        self.map_mic_vector = None
        self.time = Header()
        self.time2 = Header()
        #self.threshold = rospy.get_param("~threshold", args.threshold)
        
        self.use_async = rospy.get_param("~approximate_sync", True)
        self.subscribe_sound_direction()
        self.pub = rospy.Publisher("~output", Marker, queue_size=1)

    def subscribe_sound_direction(self):
        self.sound_direction = rospy.Subscriber("/wavdata_node_copy/max", HarkSource, self.callback_sound_direction, queue_size=1)

    def unsubscribe_sound_direction(self):
        self.sound_direction.unregister()

    def callback_sound_direction(self, sd_msg):
        # self.time2 = sd_msg.header
        # print(self.time2.stamp.to_sec())
        print("sound_callback")
        pub_msg = Marker()
        pub_msg.header = sd_msg.header
        pub_msg.header.frame_id = "map"
        pub_msg.type = 5
        pub_msg.color.r = 1.0
        pub_msg.color.a = 1.0

        pub_msg.scale.x = 0.1
        pub_msg.scale.y = 0.1
        pub_msg.scale.z = 0.1
        
        max_direction = sd_msg.src[0]
        max_point, max_azimuth, max_elevation = self.dir_to_point(max_direction)
        print("max_point:", max_point)
        self.max_point = max_point

        map_point = PointStamped()
        map_mic_point = PointStamped()

        #if self.ins_buffer[0] > self.threshold:
        map_point.header = sd_msg.header
        map_point.header.frame_id = "map"
        map_mic_point.header = sd_msg.header
        map_mic_point.header.frame_id = "map"
        #map_to_mic_coords = self.get_map_to_mic()
        map_to_mic_coords = self.get_a_to_b("map", "tamago1")
        #map to sound source音源
        map_point.point.x, map_point.point.y, map_point.point.z = map_to_mic_coords.transform_vector(max_point)
        #map to micそのもの
        map_mic_point.point.x, map_mic_point.point.y, map_mic_point.point.z = map_to_mic_coords.transform_vector(np.array([0,0,0]))

        #direction vector
        self.map_mic_vector = np.array([map_mic_point.point.x,
                                   map_mic_point.point.y,
                                   map_mic_point.point.z])
        self.direction_vector = np.array([map_point.point.x - map_mic_point.point.x ,
                                     map_point.point.y - map_mic_point.point.y ,
                                     map_point.point.z - map_mic_point.point.z])

        #rviz に描画
        pub_msg.points = []
        pub_msg.points.append(map_point.point)
        pub_msg.colors.append(ColorRGBA(r=1.0, a=1.0))
        pub_msg.points.append(map_mic_point.point)
        pub_msg.colors.append(ColorRGBA(r=1.0, a=1.0))
        #else:
        #    pub_msg.points = []

        #label = "key"
        weights = np.load(osp.join(self.rospack.get_path("action_pkg"), "gauss_data", self.label, "weights.npy"), allow_pickle=True)
        means_3d = np.load(osp.join(self.rospack.get_path("action_pkg"), "gauss_data", self.label, "means_3d.npy"), allow_pickle=True)
        covars_3d = np.load(osp.join(self.rospack.get_path("action_pkg"), "gauss_data", self.label, "covars_3d.npy"), allow_pickle=True)

        print(weights)
        d = self.map_mic_vector
        e = self.direction_vector
        k = np.arange(0, 2, 0.1)

        f = np.array([])
        for i in k:
            f = np.append(f, d+ i * e)
        #d = d.reshape(-1,3)
        #print(d.shape)
        f = f.reshape(-1, 3)
        print(f.shape)
        
        #x = np.array([0,0,0])
        #x = x.reshape(1,3)
        #print(x.shape)

        for i in range(len(weights)):
            if i == 0:
                D = weights[0] * self.gaussian(f, means_3d[0], covars_3d[0])
            else:
                D += weights[i] * self.gaussian(f, means_3d[i], covars_3d[i])
        print(D)
        print(D.max())

        self.pub.publish(pub_msg)
        #self.flag = True

    def gaussian(self, x, mu, sigma):
        #分散共分散行列の行列式
        det = np.linalg.det(sigma)
        #print(det)
        #分散共分散行列の逆行列
        inv = np.linalg.inv(sigma)
        n = x.ndim
        #print(inv.shape)

        #print((x - mu).shape)
        return np.exp(-np.diag(np.dot(np.dot((x - mu),inv),(x - mu).T))/2.0) / (np.sqrt((2 * np.pi) ** n * det))

if __name__=="__main__":
    rospy.init_node("visualize_sound_max")
    visualize_sound_max = VisualizeSoundMax("a")
    #visualize_sound_max.test1()
    rospy.spin()

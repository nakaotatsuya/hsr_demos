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

from jsk_recognition_msgs.msg import Accuracy
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
        self.pub_max = rospy.Publisher("~output_max", Marker, queue_size=1)
        self.pub_max_prob = rospy.Publisher("~prob", Accuracy, queue_size=1)

        self.gauss_max_fridge = rospy.Subscriber("/visualized_sound_pos/output_gauss_max_fridge", PointStamped, self.callback_gauss_max_fridge, queue_size=1)
        self.gauss_max_kettle = rospy.Subscriber("/visualized_sound_pos/output_gauss_max_kettle", PointStamped, self.callback_gauss_max_kettle, queue_size=1)
        self.gauss_max_microwave = rospy.Subscriber("/visualized_sound_pos/output_gauss_max_microwave", PointStamped, self.callback_gauss_max_microwave, queue_size=1)
        self.gauss_max_key = rospy.Subscriber("/visualized_sound_pos/output_gauss_max_key", PointStamped, self.callback_gauss_max_key, queue_size=1)

        self.fridge_point = None
        self.kettle_point = None
        self.microwave_point = None
        self.key_point = None
        
    def callback_gauss_max_fridge(self, msg):
        self.fridge_point = msg.point
    def callback_gauss_max_kettle(self, msg):
        self.kettle_point = msg.point
    def callback_gauss_max_microwave(self, msg):
        self.microwave_point = msg.point
    def callback_gauss_max_key(self, msg):
        self.key_point = msg.point

    def calc_distance(self):
        get_map_to_base_link = self.get_a_to_b("/map", "/base_link")
        base_link_pos = get_map_to_base_link.transform_vector(np.array([0,0,0]))

        if self.fridge_point is not None:
            fridge_d = np.sqrt( (base_link_pos[0] - self.fridge_point.x )**2 + (base_link_pos[1] - self.fridge_point.y )**2 )
            #print("fridge:", fridge_d)

        if self.kettle_point is not None:
            kettle_d = np.sqrt( (base_link_pos[0] - self.kettle_point.x )**2 + (base_link_pos[1] - self.kettle_point.y )**2 )
            #print("kettle:", kettle_d)

        if self.microwave_point is not None:
            microwave_d = np.sqrt( (base_link_pos[0] - self.microwave_point.x )**2 + (base_link_pos[1] - self.microwave_point.y )**2 )
            #print("microwave:", microwave_d)

        if self.key_point is not None:
            key_d = np.sqrt( (base_link_pos[0] - self.key_point.x )**2 + (base_link_pos[1] - self.key_point.y )**2 )
            #print("key:", key_d)

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

        pub_msg.scale.x = 0.01
        pub_msg.scale.y = 0.01
        pub_msg.scale.z = 0.01

        pub_max_msg = Marker()
        pub_max_msg.header = sd_msg.header
        pub_max_msg.header.frame_id = "map"
        pub_max_msg.type = 8
        pub_max_msg.color.g = 1.0
        pub_max_msg.color.a = 1.0
        pub_max_msg.scale.x = 0.1
        pub_max_msg.scale.y = 0.1
        pub_max_msg.scale.z = 0.1
        
        max_direction = sd_msg.src[0]
        max_point, max_azimuth, max_elevation = self.dir_to_point(max_direction)
        #print("max_point:", max_point)
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

        #else:
        #    pub_msg.points = []

        #label = "key"
        weights = np.load(osp.join(self.rospack.get_path("action_pkg"), "gauss_data", self.label, "weights.npy"), allow_pickle=True)
        means_3d = np.load(osp.join(self.rospack.get_path("action_pkg"), "gauss_data", self.label, "means_3d.npy"), allow_pickle=True)
        covars_3d = np.load(osp.join(self.rospack.get_path("action_pkg"), "gauss_data", self.label, "covars_3d.npy"), allow_pickle=True)

        #print(weights)
        d = self.map_mic_vector
        e = self.direction_vector
        k = np.arange(0, 3, 0.1)

        f = np.array([])
        for i in k:
            f = np.append(f, d+ i * e)
        #d = d.reshape(-1,3)
        #print(d.shape)
        f = f.reshape(-1, 3)
        #print(f.shape)
        
        #x = np.array([0,0,0])
        #x = x.reshape(1,3)
        #print(x.shape)

        for i in range(len(weights)):
            if i == 0:
                D = weights[0] * self.gaussian(f, means_3d[0], covars_3d[0])
            else:
                D += weights[i] * self.gaussian(f, means_3d[i], covars_3d[i])
        #print(D)
        #print(D.max())

        max_idx = D.argmax()

        pub_max_prob_msg = Accuracy()
        pub_max_prob_msg.header = sd_msg.header
        pub_max_prob_msg.header.frame_id = "map"
        pub_max_prob_msg.accuracy = D.max()

        #rviz に描画
        pub_msg.points = []
        #pub_msg.points.append(map_point.point)
        pub_msg.points.append(Point(x=f[29][0], y=f[29][1], z=f[29][2]))
        pub_msg.colors.append(ColorRGBA(r=1.0, a=1.0))
        pub_msg.points.append(map_mic_point.point)
        pub_msg.colors.append(ColorRGBA(r=1.0, a=1.0))
        
        pub_max_msg.points = []
        pub_max_msg.points.append(Point(x=f[max_idx][0], y=f[max_idx][1], z=f[max_idx][2]))
        print(f[max_idx])
        pub_max_msg.colors.append(ColorRGBA(b=1.0, a=1.0))
        
        self.pub.publish(pub_msg)
        self.pub_max.publish(pub_max_msg)
        self.pub_max_prob.publish(pub_max_prob_msg)
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

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.calc_distance()
            rate.sleep()
            
if __name__=="__main__":
    rospy.init_node("visualize_sound_max")
    visualize_sound_max = VisualizeSoundMax("a")
    #visualize_sound_max.test1()
    #visualize_sound_max.calc_distance()
    #rospy.spin()
    visualize_sound_max.run()

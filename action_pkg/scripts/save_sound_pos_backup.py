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

import argparse

class SaveSoundPos(RobotAction, SpotMixin):
    def __init__(self, name):
        super(SaveSoundPos, self).__init__(name)
        #self.move_base = actionlib.SimpleActionClient(
        #    '/move_base/move', MoveBaseAction)
        #self.move_base.wait_for_server()
        #self.whole_body = self.robot.get("whole_body")
        #self.omni_base = self.robot.get('omni_base')

        rospack = rospkg.RosPack()
        parser = argparse.ArgumentParser()
        parser.add_argument("-l", "--label", type=str, default="test")
        args = parser.parse_args()

        self.label = rospy.get_param("~label", args.label)
        self.save_dir = osp.join(rospack.get_path(
            'action_pkg'), "sound_pos_data", self.label)
        if not osp.exists(self.save_dir):
            makedirs(self.save_dir)
        self.listener = tf.TransformListener()

        self.flag = False
        self.ins_buffer = np.array([])
        self.map_mic_vector = None
        self.time = Header()
        self.time2 = Header()
        self.threshold = rospy.get_param("~threshold", 1.5)
        
        self.use_async = rospy.get_param("~approximate_sync", True)
        self.pub = rospy.Publisher("~output", Marker, queue_size=1)

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
        #print(self.ins_buffer[0:8])
        #print(self.ins_buffer)

        if np.all(self.ins_buffer[0:8] >= self.threshold):
            self.time = ins_msg.header
            self.flag = True

    def subscribe_sound_direction(self):
        self.sound_direction = rospy.Subscriber("/wavdata_node_copy/max", HarkSource, self.callback_sound_direction, queue_size=1)

    def unsubscribe_sound_direction(self):
        self.sound_direction.unregister()

    def callback_sound_direction(self, sd_msg):
        self.time2 = sd_msg.header
        print(self.time2.stamp.to_sec())
        print("sound_callback")
        pub_msg = Marker()
        pub_msg.header = sd_msg.header
        pub_msg.header.frame_id = "map"
        pub_msg.type = 4
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
        map_to_mic_coords = self.get_map_to_mic()
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
        pub_msg.points.append(map_mic_point.point)
        #else:
        #    pub_msg.points = []

        #self.pub.publish(pub_msg)
        #self.flag = True

    def test1(self):
        self.subscribe_in_sound()
        self.subscribe_sound_direction()
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while not self.flag:
            rate.sleep()
            if (rospy.Time.now() - start_time).to_sec() > 10.0:
                print("aaaaaaaaaaaaaaaaaaa")
                return True
        self.unsubscribe_in_sound()

        print("time:", self.time.stamp.to_sec())
        print("now:", rospy.Time.now().to_sec())
        max_point_list = []
        direction_vector_list = []
        while self.time.stamp.to_sec() + 0.6 > rospy.Time.now().to_sec():
            rate.sleep()
            max_point_list.append(self.max_point)
            direction_vector_list.append(self.direction_vector)
            if (rospy.Time.now() - start_time).to_sec() > 20.0:
                return True
        self.unsubscribe_sound_direction()

        print("len:", max_point_list)
        print("len2:", direction_vector_list)
        # self.flag = False
        # self.subscribe_sound_direction()
        # rate = rospy.Rate(10)
        # start_time = rospy.Time.now()
        # while not self.flag:
        #     rate.sleep()
        #     if (rospy.Time.now() - start_time).to_sec() > 10.0:
        #         print("gggggggggggggggggg")
        #         return True
        # self.unsubscribe_sound_direction()

        # p + s*v (pがself.map_mic_vector , vがself.direction_vector, sは媒介変数)
        # これらをsaveする
        file_num = len(listdir(self.save_dir)) + 1
        if osp.isfile(osp.join(self.save_dir, "center.npy")):
            file_num -= 1
        print(self.direction_vector)
        pv = np.array([self.map_mic_vector, self.direction_vector])
        np.save(osp.join(self.save_dir, '{:0=5d}.npy'.format(file_num)), pv)

    # def subscribe_sound(self):
    #     sound_direction = message_filters.Subscriber("/wavdata_node_copy/max", HarkSource, queue_size=1)
    #     #in_sound = message_filters.Subscriber("/sound_detector_volume/in_sound", InSound)
    #     in_sound = message_filters.Subscriber("/tamago1/harkwave", HarkWave, queue_size=1, buff_size=2**24)
    #     self.subs = [sound_direction, in_sound]
    #     if self.use_async:
    #         sync = message_filters.ApproximateTimeSynchronizer(self.subs, queue_size=1, slop=0.1)
    #     else:
    #         sync = message_filters.TimeSynchronizer(self.subs, queue_size=1)
    #     sync.registerCallback(self.sound_callback)

    # def unsubscribe_sound(self):
    #     for sub in self.subs:
    #         sub.unregister()

    # def sound_callback(self, sd_msg, ins_msg):
    #     print("sound_callback")
    #     pub_msg = Marker()
    #     pub_msg.header = sd_msg.header
    #     pub_msg.header.frame_id = "map"
    #     pub_msg.type = 4
    #     pub_msg.color.r = 1.0
    #     pub_msg.color.a = 1.0

    #     pub_msg.scale.x = 0.1
    #     pub_msg.scale.y = 0.1
    #     pub_msg.scale.z = 0.1

    #     max_direction = sd_msg.src[0]
    #     max_point, max_azimuth, max_elevation = self.dir_to_point(max_direction)
    #     print("max_point:", max_point)

    #     map_point = PointStamped()
    #     map_mic_point = PointStamped()

    #     #print(ins_msg.src[0].wavedata)
    #     for i in range(len(ins_msg.src)):
    #         test = np.array(ins_msg.src[i].wavedata)
    #         #print(test.max())
        # if ins_msg.src:
        #     map_point.header = sd_msg.header
        #     map_point.header.frame_id = "map"
        #     map_mic_point.header = sd_msg.header
        #     map_mic_point.header.frame_id = "map"
        #     map_to_mic_coords = self.get_map_to_mic()
        #     #map to sound source音源
        #     map_point.point.x, map_point.point.y, map_point.point.z = map_to_mic_coords.transform_vector(max_point)
        #     #map to micそのもの
        #     map_mic_point.point.x, map_mic_point.point.y, map_mic_point.point.z = map_to_mic_coords.transform_vector(np.array([0,0,0]))

        #     #direction vector
        #     map_mic_vector = np.array([map_mic_point.point.x,
        #                                map_mic_point.point.y,
        #                                map_mic_point.point.z])
        #     direction_vector = np.array([map_point.point.x - map_mic_point.point.x ,
        #                                  map_point.point.y - map_mic_point.point.y ,
        #                                  map_point.point.z - map_mic_point.point.z])

        #     # p + s*v (pがself.map_mic_point , vがdirection_vector, sは媒介変数)
        #     # これらをsaveする
        #     file_num = len(listdir(self.save_dir)) +  1
        #     pv = np.array([map_mic_vector, direction_vector])
        #     np.save(osp.join(self.save_dir, '{:0=5d}.npy'.format(file_num)), pv)

        #     #rviz に描画
        #     pub_msg.points = []
        #     pub_msg.points.append(map_point.point)
        #     pub_msg.points.append(map_mic_point.point)
        #     self.flag = True
        # else:
        #     pub_msg.points = []

        # self.pub.publish(pub_msg)

    def get_map_to_mic(self):
        import skrobot
        succeed = False
        while not succeed:
            try:
                trans, rot = self.listener.lookupTransform(
                    "map", "tamago1",
                    rospy.Time(0))
                succeed = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        print("get map to mic")
        c = skrobot.coordinates.Coordinates(
            pos=trans, rot=skrobot.coordinates.math.xyzw2wxyz(rot))
        return c

    def dir_to_point(self, direction):
        #direction's type is supposed to be hark_msgs/HarkSource."
        print(direction.azimuth)
        print(direction.elevation)
        azimuth = np.radians(direction.azimuth)
        elevation = np.radians(direction.elevation)
        x = np.cos(elevation) * np.cos(azimuth)
        y = np.cos(elevation) * np.sin(azimuth)
        z = np.sin(elevation)
        point = np.array([x,y,z])
        return point, azimuth, elevation

if __name__=="__main__":
    rospy.init_node("save_sound_pos")
    save_sound_pos = SaveSoundPos("a")
    save_sound_pos.test1()
    #rospy.spin()

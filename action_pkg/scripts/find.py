#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import sys
import tf
import tf.transformations
import numpy as np
import actionlib
import rospy
import hsrb_autocharge.msg
import geometry_msgs.msg
import tmc_msgs.msg
import std_msgs.msg
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import message_filters
from tmc_eus_py.coordinates import Coordinates
from geometry_msgs.msg import PoseArray, PoseStamped
from geometry_msgs.msg import WrenchStamped
import hsrb_interface
from hsrb_interface import geometry
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

from jsk_recognition_msgs.msg import LabelArray, BoundingBoxArray, ClassificationResult, PeoplePoseArray
from hark_msgs.msg import HarkSource
from sound_classification.msg import InSound

from jsk_hsr_startup.robot_action import RobotAction
from jsk_hsr_startup.spot_mixin import SpotMixin
from jsk_hsr_startup.tmc_speak import speak_jp

from geometry_msgs.msg import PointStamped, Point, Quaternion

#TODO
# 音が聞こえる方にロボットが近づく
# ものを認識して実際に音が聞こえるかどうかを再確認　例えばものが3つくらい見える環境にいた場合、どれが対象の物体かわかりづらいので。
# 合っていれば、その音をデータとしてためていく。 →　結局何を学習することになるか?? 聞いた音
# これで探しものができる
# 次に、死角などで見えない場所にあるとき、棚をあけるなど。 音は聞こえているけど、ものが見えないとき。
# 「鍵を探して」→人間がスマホから音を鳴らす→ロボットが音がなっているところにいく。 なんかおかしい。。

class Find(RobotAction, SpotMixin):
    def __init__(self, name, **kwargs):
        super(Find, self).__init__(name, **kwargs)
        self.move_base = actionlib.SimpleActionClient(
            '/move_base/move', MoveBaseAction)
        self.move_base.wait_for_server()
        self.whole_body = self.robot.get("whole_body")
        self.omni_base = self.robot.get('omni_base')
        self.point = None

        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        self.azimuth = 0.0
        self.elevation = 0.0

        #self.subscribe_sound()

    def subscribe_sound(self):
        sound_direction = message_filters.Subscriber("/wavdata_node_copy/max", HarkSource)
        in_sound = message_filters.Subscriber("/sound_detector_volume/in_sound", InSound)
        use_async = rospy.get_param("~approximate_sync", True)
        queue_size = rospy.get_param("~queue_size", 1)
        self.subs = [sound_direction, in_sound]
        if use_async:
            slop = rospy.get_param("~slop", 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(self.subs, queue_size, slop)
        else:
            sync = message_filters.TimeSynchronizer(self.subs, queue_size)
        sync.registerCallback(self.sound_callback)

    def unsubscribe_sound(self):
        for sub in self.subs:
            sub.unregister()

    def subscribe_box(self):
        sub_label_array = message_filters.Subscriber(
            "/mask_rcnn_73b2_kitchen/output/labels")

    def sound_callback(self, sd_msg, ins_msg):
        max_direction = sd_msg.src[0]
        max_point, max_azimuth, max_elevation = self.dir_to_point(max_direction)
        print("max_point:", max_point)
        self.point = PointStamped()
        if max_point and ins_msg.in_sound:
            self.point.header.frame_id = "base_link"
            #point.point.x, point.point.y, point.point.z = max_point
            base_link_to_mic_coords = self.get_base_link_to_mic()
            self.point.point.x, self.point.point.y, self.point.point.z = base_link_to_mic_coords.transform_vector(max_point)

            self.azimuth = np.arctan(self.point.point.y / self.point.point.x)
            if max_point[0] < 0.0:
                self.azimuth += np.pi
            self.elevation = np.arctan(
                self.point.point.z / np.sqrt(self.point.point.x ** 2 + self.point.point.y ** 2))
            #self.azimuth = max_azimuth
            #self.elevation = max_elevation
            print(self.point.point.x, self.point.point.y, self.point.point.z, self.azimuth, self.elevation)
        else:
            self.point = None

    def dir_to_point(self, direction):
        azimuth = np.radians(direction.azimuth)
        elevation = np.radians(direction.elevation)
        x = np.cos(elevation) * np.cos(azimuth)
        y = np.cos(elevation) * np.sin(azimuth)
        z = np.sin(elevation)
        point = [x,y,z]
        return point, azimuth, elevation

    def get_base_link_to_mic(self):
        import skrobot
        succeed = False
        while not succeed:
            try:
                trans, rot = self.listener.lookupTransform(
                    "base_link", "tamago1",
                    rospy.Time(0))
                succeed = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        print("get base_link to mic")
        c = skrobot.coordinates.Coordinates(
            pos=trans, rot=skrobot.coordinates.math.xyzw2wxyz(rot))
        return c

    def move_to_sound_source(self, wait=True):
        self.whole_body.move_to_neutral()

        self.move_to("/eng8/6f/room610-center-table-side", wait=True)

        #hearing sounds
        self.subscribe_sound()
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        print("start")
        while (self.point == None) or (self.azimuth == 0.0):
            rate.sleep()
            if (rospy.Time.now() - start_time).to_sec() > 10.0:
                print("I couldn't hear any sound.")
                return True
        self.unsubscribe_sound()
        #
        
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id ="base_link"
        pose.pose.position = Point(0,0,0)
        quat = tf.transformations.quaternion_from_euler(0, 0, self.azimuth)
        pose.pose.orientation = Quaternion(*quat)
        print(pose)

        goal = MoveBaseGoal()
        goal.target_pose = pose
        self.move_base.send_goal(goal)
        if wait is True:
            self.move_base.wait_for_result()
            action_state = self.move_base.get_state()
            if action_state != GoalStatus.SUCCEEDED:
                rospy.loginfo('failed move_base: {}'.format(action_state))
                rospy.loginfo('failed move_base: {}'.format(
                    self.move_base.get_result()))
                rospy.loginfo("I couldn't move.")
                #speak_jp('目的地に移動できませんでした。')
                return False
            return True

        print(self.point.point.x, self.point.point.y, self.point.point.z)
        gaze_pose = geometry.vector3(self.point.point.x, self.point.point.y, self.point.point.z)
        self.whole_body.gaze_point(point=gaze_pose, ref_frame_id="base_link")

        #looking for objects
        self.target_boxes = []
        self.subscribe_box()
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while len(self.target_boxes) == 0:
            #if head_tilt_joint < -1.0:
            #    break
            rate.sleep()
            if (rospy.Time.now() - start_time).to_sec() > 3.0:
                #head_tilt_joint += -0.1
                #start_time = rospy.Time.now()
                #whole_body.move_to_joint_positions(
                #    {"head_tilt_joint": head_tilt_joint})
                print("I couldn't find objects.")
        self.unsubscribe_box()
        #
        
        #gradually proceed
        #TODO
        
        return self.move_base

    def move_to(self, spot_name, wait=True):
        pose_stamped = self.lookup_spot(spot_name)
        if pose_stamped.header.frame_id == "":
            rospy.loginfo("I don't know the spot name.")
            return
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.z = 0.0

        goal = MoveBaseGoal()
        goal.target_pose = pose_stamped
        self.move_base.send_goal(goal)
        if wait is True:
            self.move_base.wait_for_result()
            action_state = self.move_base.get_state()
            if action_state != GoalStatus.SUCCEEDED:
                rospy.loginfo("failed move_base: {}".format(action_state))
                rospy.loginfo("failed move_base: {}".format(
                    self.move_base.get_result()))
                return False
            return True
        return self.move_base

if __name__=="__main__":
    rospy.init_node("find")
    find = Find("a")
    find.move_to_sound_source()
    rospy.spin()

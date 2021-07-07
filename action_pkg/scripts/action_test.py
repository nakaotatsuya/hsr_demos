#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import sys
import tf
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

from jsk_hsr_startup.robot_action import RobotAction
from jsk_hsr_startup.spot_mixin import SpotMixin
from jsk_hsr_startup.tmc_speak import speak_jp

from geometry_msgs.msg import PolygonStamped, PointStamped

class FindPerson(RobotAction, SpotMixin):
    def __init__(self, name, **kwargs):
        super(FindPerson, self).__init__(name, **kwargs)
        #RobotAction.__init__(self, 'take_plastic_bottle')
        self.move_base = actionlib.SimpleActionClient(
            '/move_base/move', MoveBaseAction)
        self.move_base.wait_for_server()
        self.whole_body = self.robot.get("whole_body")
        self.omni_base = self.robot.get('omni_base')
        self.sc = np.array([])
        self.sc_len = 2
        self.point = None
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        #self.subscribe_people_pose()

    def on_start(self):
        sound_direction = message_filters.Subscriber("/wavdata_node_copy/max", HarkSource)
        sound_class = message_filters.Subscriber("/sound_classifier/output", ClassificationResult)
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

    def subscribe_people_pose(self):
        self.people_pose_sub = rospy.Subscriber("people_pose_estimation_2d/pose", PeoplePoseArray, self.people_pose_callback)

    def unsubscribe_people_pose(self):
        self.people_pose_sub.unregister()

    def people_pose_callback(self, msg):
        #print(msg.poses[0])
        #subscribe nose point
        if len(msg.poses):
            x = msg.poses[0].poses[0].position.x
            y = msg.poses[0].poses[0].position.y
            z = msg.poses[0].poses[0].position.z
            if (x == 0.0 and y == 0.0) and z==0.0:
                return
            self.br.sendTransform((x,y,z),
                                  (0.0, 0.0, 0.0, 1.0),
                                  rospy.Time.now(),
                                  "people_spot",
                                  "head_rgbd_sensor_rgb_frame")
            # self.br.sendTransform((x, y, z-0.5),
            #                       (0.0, 0.0, 0.0, 1.0),
            #                       rospy.Time.now(),
            #                       "people_front_spot",
            #                       "head_rgbd_sensor_rgb_frame")

            quat = tf.transformations.quaternion_from_euler(math.pi/2.0,-math.pi/2.0,0)
            self.br.sendTransform((x, y, z-0.5),
                                  quat,
                                  rospy.Time.now(),
                                  "people_front_spot_2",
                                  "head_rgbd_sensor_rgb_frame")

    def on_end(self):
        for sub in self.subs:
            sub.unregister()
        self.timer.shutdown()
        
    def timer_cb(self, args=None):
        print(self.sc)
        if self.point and all([e == "clap\n" for e in self.sc]):
            try:
                #self.look_at(self.point)
                #pose = geometry.vector3(self.point.point.x, self.point.point.y, self.point.point.z)
                pose = geometry.vector3(self.point.point.x, self.point.point.y, 0)
                #print(pose)
                self.whole_body.gaze_point(point=pose, ref_frame_id="tamago1")
                #self.whole_body.gaze_point(point=pose, ref_frame_id=self.point.header.frame_id)
            except geometry.exceptions.MotionPlanningError:
                pass

    def _callback(self, sd_msg, sc_msg):
        max_direction = sd_msg.src[0]
        max_point = self.dir_to_point(max_direction)
        #print(max_point)
        self.sc = np.append(self.sc, sc_msg.label_names[0])
        self.sc = self.sc[-self.sc_len:]
        point = PointStamped()
        if max_point:
            point.header.frame_id = sd_msg.header.frame_id
            point.point.x = max_point[0]
            point.point.y = max_point[1]
            point.point.z = max_point[2]
        else:
            point = None
        self.point = point
        #print(self.point)

    def dir_to_point(self, direction):
        x = np.cos(np.radians(direction.elevation)) * np.cos(np.radians(direction.azimuth))
        y = np.cos(np.radians(direction.elevation)) * np.sin(np.radians(direction.azimuth))
        z = np.sin(np.radians(direction.elevation))
        # print("mic:")
        # print(x_mic, y_mic, z_mic)
        # camera_to_mic_coords = self.get_camera_to_mic()
        # x,y,z = camera_to_mic_coords.transform_vector(
        #     (x_mic, y_mic, z_mic))
        point = (x,y,z)
        #print("camera:")
        #print(point)
        return point

    # def get_camera_to_mic(self):
    #     import skrobot
    #     succeed = False
    #     while not succeed:
    #         try:
    #             trans, rot = self.listener.lookupTransform(
    #                 "head_rgbd_sensor_rgb_frame",
    #                 "tamago1",
    #                 rospy.Time(0))
    #             succeed = True
    #         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #             continue
    #     print("get mic camera end")
    #     c = skrobot.coordinates.Coordinates(
    #         pos=trans,
    #         rot=skrobot.coordinates.math.xyzw2wxyz(rot))
    #     return c

    # def move_to(self, spot_name, wait=True):
    #     pose_stamped = self.lookup_spot(spot_name)
    #     if pose_stamped.header.frame_id == '':
    #         #speak_jp('位置がわかりませんでした。')
    #         return

    #     pose_stamped.header.stamp = rospy.Time.now()
    #     pose_stamped.pose.position.z = 0.0
    #     pose_stamped.header.frame_id = 'map'

    #     #print(pose_stamped)
        
    #     goal = MoveBaseGoal()
    #     goal.target_pose = pose_stamped
    #     self.move_base.send_goal(goal)
    #     if wait is True:
    #         self.move_base.wait_for_result()

    #         action_state = self.move_base.get_state()
    #         if action_state != GoalStatus.SUCCEEDED:
    #             rospy.loginfo('failed move_base: {}'.format(action_state))
    #             rospy.loginfo('failed move_base: {}'.format(
    #                 self.move_base.get_result()))
    #             #speak_jp('目的地に移動できませんでした。')
    #             return False
    #         return True
    #     return self.move_base

    def move_to_point(self, ps, wait=True):
        pose_stamped = ps
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.z = 0.0
        pose_stamped.header.frame_id = "map"
        print(pose_stamped)

        goal = MoveBaseGoal()
        goal.target_pose = pose_stamped
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
        return self.move_base

    def get_map_to_people(self):
        succeed = False
        while not succeed:
            try:
                trans, rot = self.listener.lookupTransform(
                    "map",
                    "people_front_spot_2",
                    rospy.Time(0))
                succeed = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        ps = PoseStamped()
        ps.pose.position.x = trans[0]
        ps.pose.position.y = trans[1]
        ps.pose.position.z = trans[2]

        ps.pose.orientation.x = rot[0]
        ps.pose.orientation.y = rot[1]
        ps.pose.orientation.z = rot[2]
        ps.pose.orientation.w = rot[3]
        print(ps)
        return ps

    def move_to_people(self):
        people_pose = None
        self.subscribe_people_pose()
        rate = rospy.Rate(10)
        start = rospy.Time.now()
        while (people_pose is None) and not rospy.is_shutdown():
            people_pose = self.get_map_to_people()
            #if (rospy.Time.now() - start).to_sec() > 1.0:
            #    start = rospy.Time.now()
            #rate.sleep()
        if people_pose is None:
            rospy.loginfo("can't find people")
            return
        self.unsubscribe_people_pose()

        retry = False
        #undock
        self.omni_base.go_rel(0,0, -0.3491)
        self.omni_base.go_rel(0, 0.2, 0)

        #self.omni_base.go_pose(geometry.pose(x=0.2), 100.0, ref_frame_id="base_link")
        self.move_to_point(people_pose)

    def run(self, args=None):
        first = True
        while not rospy.is_shutdown():
            if all([e == "clap\n" for e in self.sc]) and first:
                print("true")
                first = False
                self.move_to_people()
            rospy.sleep(1)

if __name__ == "__main__":
    rospy.init_node("action_test")
    action = FindPerson("hear sounds")
    #action.move_to("/eng8/6f/room610-wrs-shelf", wait=True)
    #action.move_to_people()
    #rospy.spin()
    action()

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import hsrb_interface
import numpy as np
from hsrb_interface import geometry
import locale
from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal
from std_msgs.msg import ColorRGBA

from jsk_hsr_startup.robot_action import RobotAction
from jsk_hsr_startup.spot_mixin import SpotMixin

import tf
import math

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

import skrobot

class MyRobot(RobotAction, SpotMixin):
    def __init__(self, name, **kwargs):
        super(MyRobot, self).__init__(name, **kwargs)
        self.listener = tf.TransformListener()
        self.move_base = actionlib.SimpleActionClient(
            '/move_base/move', MoveBaseAction)
        self.move_base.wait_for_server()

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
        #point = [x,y,z]
        return point, azimuth, elevation

    def point_to_dir(self, point):
        #point's type is supposed to be geometry_msgs/PointStamped
        azimuth = np.arctan(point.point.y / point.point.x)
        if point.point.x < 0.0:
            azimuth += np.pi
        elevation = np.arctan(
            point.point.z / np.sqrt(point.point.x ** 2 + point.point.y ** 2))
        return azimuth, elevation

    def get_a_to_b(self, a, b):
        succeed = False
        while not succeed:
            try:
                trans, rot = self.listener.lookupTransform(
                    a, b, rospy.Time(0))
                succeed = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        c = skrobot.coordinates.Coordinates(
            pos=trans, rot=skrobot.coordinates.math.xyzw2wxyz(rot))
        return c

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
    rospy.init_node("my_robot")
    a = MyRobot("test")

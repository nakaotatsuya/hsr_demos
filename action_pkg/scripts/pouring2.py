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

from jsk_recognition_msgs.msg import LabelArray, BoundingBoxArray, ClassificationResult, PeoplePoseArray, Accuracy
from hark_msgs.msg import HarkSource

from jsk_hsr_startup.robot_action import RobotAction
from jsk_hsr_startup.spot_mixin import SpotMixin
from jsk_hsr_startup.tmc_speak import speak_jp

from geometry_msgs.msg import PolygonStamped, PointStamped
from my_robot import MyRobot


class Pouring(MyRobot):
    def __init__(self, name, **kwargs):
        super(Pouring, self).__init__(name, **kwargs)
        #self.move_base = actionlib.SimpleActionClient(
        #    '/move_base/move', MoveBaseAction)
        #self.move_base.wait_for_server()
        self.whole_body = self.robot.get("whole_body")
        #self.omni_base = self.robot.get('omni_base')
        self.gripper = self.robot.try_get("gripper")

        self.sound_flow = np.array([])
        self.sound_flow_len = 100
        print("init")

    def subscribe(self):
        #self.sound_class = rospy.Subscriber("/sound_classifier/output", ClassificationResult, self._callback, queue_size=1)
        self.sound_class = rospy.Subscriber("/sound_classifier/output/criteria", Accuracy, self._callback, queue_size=1)

    def unsubscribe(self):
        self.sound_class.unregister()

    def _callback(self, msg):
        #print(msg.label_names)
        #self.sound_flow = np.append(self.sound_flow, msg.label_names[0])
        self.sound_flow = np.append(self.sound_flow, msg.accuracy)
        self.sound_flow = self.sound_flow[-self.sound_flow_len:]
        #print(self.sound_flow)
        #print(len(self.sound_flow))
        # if msg.label_names == "no_sound":
        #     self.pour()
        # else msg.label_names == "high":
        #     self.reverse()
        
    def pour(self, roll):
        #self.whole_body.move_end_effector_by_arc(geometry.pose(x=0.0, y=0.0, z=0.0), math.radians(5.0), ref_frame_id="hand_palm_link")
        self.whole_body.move_to_joint_positions({"wrist_roll_joint": roll})

    def pour_little(self):
        #self.whole_body.move_end_effector_by_arc(geometry.pose(x=0.0, y=0.0, z=0.0), math.radians(1.0), ref_frame_id="hand_palm_link")
        self.whole_body.move_to_joint_positions({"wrist_roll_joint": 0.03})

    def action(self):
        while not rospy.is_shutdown():
            retry = False
            try:
                #self.whole_body.move_to_joint_positions({"arm_lift_joint": 0.15})
                #self.gripper.apply_force(0.1)
                roll = 0.0
                self.subscribe()
                rate = rospy.Rate(10)
                start_time = rospy.Time.now()
                #print("a")
                while not rospy.is_shutdown():
                    rate.sleep()
                    if all([(x >= 0.35)  for x in self.sound_flow[0:30]]) and len(self.sound_flow)==self.sound_flow_len:
                        roll += 0.01
                        print("aaaa")
                        self.pour(roll)
                    if all([(x <= 0.10) for x in self.sound_flow[0:30]]) and len(self.sound_flow)==self.sound_flow_len:
                        roll += 0.0015
                        self.pour(roll)
                    #    self.pour_little()
                    if all([((x > 0.10) and (x < 0.30)) for x in self.sound_flow[0:9]]) and len(self.sound_flow)==self.sound_flow_len:
                        self.whole_body.move_to_joint_positions({"wrist_roll_joint": 0})
                        break
                        return True
                    #if (rospy.Time.now() - start_time).to_sec() > 120.0:
                    #    break
                self.unsubscribe()
                #retry = False
            except hsrb_interface.exceptions.GripperError:
                retry = True
            except hsrb_interface.exceptions.FollowTrajectoryError:
                retry = True
            except hsrb_interface.exceptions.MotionPlanningError:
                retry = True
            if retry:
                continue
            break
        print("eee")

if __name__ == "__main__":
    rospy.init_node("pouring")
    pouring = Pouring("pour")
    pouring.action()
    rospy.spin()

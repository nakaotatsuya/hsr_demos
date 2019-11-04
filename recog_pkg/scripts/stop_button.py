#!/usr/bin/env python

import rospy
import hsrb_interface
from hsrb_interface import geometry
from hsrb_interface import exceptions
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Quaternion
#from recog_pkg.msg import Camera3d
import tf2_ros
import tf_conversions
import math
import numpy as np
from std_msgs.msg import Float32, Int32
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from jsk_recognition_msgs.msg import LabelArray, RectArray
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
import cv_bridge
from recog_pkg.msg import Camera3d, HandCamera2d
import copy
import tmc_eus_py
from tmc_eus_py.coordinates import Coordinates
from agent import Agent
from std_srvs.srv import Empty, EmptyResponse

class StopButton():
    def __init__(self): #agent = Agent()
        self.agent=Agent()
        self.flag = False
        print("ccc")
        rospy.Subscriber("/hand_camera_2d_point", HandCamera2d, self.cb, queue_size=100)
        rospy.Service("use_hand_camera", Empty, self.use_hand_camera_cb)

    def use_hand_camera_cb(self, req):
        print("eee")
        self.flag = True
        return EmptyResponse()

    def move(self):
        r = rospy.Rate(0.33)
        while not rospy.is_shutdown():
            print("aaa-----aaa")
            if self.agent.switch == 0:
                if self.agent.u > self.agent.u_high:
                    #self.whole_body.move_end_effector_pose(geometry.pose(y=-0.02), ref_frame_id="hand_palm_link")
                    self.agent.omni_base.go_rel(x=-0.017)
                elif self.agent.u < self.agent.u_low:
                    #self.whole_body.move_end_effector_pose(geometry.pose(y=0.02), ref_frame_id="hand_palm_link")
                    self.agent.omni_base.go_rel(x=0.017)

                if self.agent.v > self.agent.v_high:
                    #self.agent.whole_body.move_end_effector_pose(geometry.pose(x=-0.02), ref_frame_id="hand_palm_link")
                    self.agent.omni_base.go_rel(y=0.017)
                elif self.agent.v < self.agent.v_low:
                    #self.agent.whole_body.move_end_effector_pose(geometry.pose(x=0.02), ref_frame_id="hand_palm_link")
                    self.agent.omni_base.go_rel(y=-0.017)

                if ((self.agent.u >= self.agent.u_low and self.agent.u <= self.agent.u_high) and (self.agent.v >= self.agent.v_low and self.agent.v <= self.agent.v_high)):
                    self.agent.switch = 1
                else:
                    pass

            elif self.agent.switch == 1:
                self.agent.whole_body.impedance_config = "compliance_hard"
                #if 4 not in self.agent.freq_list_replace:
                print("aaaaa")
                self.agent.gripper.apply_force(1.0)
                self.agent.whole_body.move_end_effector_by_line((0,0,1), 0.06)
                self.agent.gripper.command(1.0)
                self.agent.switch = 0
            r.sleep()

    def cb(self, msg):
        print("bbb")
        self.agent.u = msg.x
        self.agent.v = msg.y
        if self.flag:
            self.move()
        
if __name__ == "__main__":
    rospy.init_node("stop_button")
    stop = StopButton()
    rospy.spin()


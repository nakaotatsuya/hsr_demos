#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
import hsrb_interface
from hsrb_interface import geometry
from hsrb_interface import exceptions
from geometry_msgs.msg import PoseStamped
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
from recog_pkg.msg import Camera3d
import copy

def init():
    robot = hsrb_interface.Robot()
    omni_base = robot.get("omni_base")
    whole_body = robot.get("whole_body")
    gripper = robot.get("gripper")

def go_to_kitchen():
    kitchen_pose = geometry.Pose(pos=geometry.Vector3(x=3.11, y=1.21, z=0.0), ori=geometry.Quaternion(x=0.0, y=0.0, z=-0.01023836327822357, w=0.9999475865851083))
    omni_base.go_pose(kitchen_pose)

def go_to_yakan():
    yakan_pose = geometry.Pose(pos=geometry.Vector3(x=3.108781824048564, y=0.037752328711472494, z=0.0), ori=geometry.Quaternion(x=0.0, y=0.0, z=0.013848295774654074, w=0.9999041077544075))
    omni_base.go_pose(yakan_pose)
    
if __name__ == "__main__":
    init()
    #go_to_kitchen()
    go_to_yakan()

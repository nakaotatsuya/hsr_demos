#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray
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
from recog_pkg.msg import Camera3d
import copy
import tmc_eus_py
from tmc_eus_py.coordinates import Coordinates
from agent import Agent
from stop_button import StopButton
from std_srvs.srv import Empty, EmptyResponse

if __name__ == "__main__":
    rospy.init_node("kettle")
    rospy.wait_for_service("use_hand_camera")
    agent = Agent()
    agent.find_stop_button_pose()
    rospy.sleep(5)
    agent.stop_yakan_solve_ik()
    rospy.sleep(5)
    print("aaa")
    use_hand_camera = rospy.ServiceProxy("use_hand_camera", Empty)
    use_hand_camera()
    print("ddd")
    #agent.hand_done = True
    #agent.go_to_stop_button()
    #agent.find_stop_button_pose()
    #rospy.sleep(5)
    #agent.stop_yakan_solve_ik()
    rospy.spin()

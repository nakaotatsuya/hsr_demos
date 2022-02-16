#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import math
import sys
import actionlib

import rospy
import rospkg
import numpy as np
import message_filters
import sympy
import tf
import tf.transformations
from os import makedirs, listdir
from os import path as osp

from tmc_eus_py.coordinates import Coordinates
from visualization_msgs.msg import Marker
from hark_msgs.msg import HarkSource
from sound_classification.msg import InSound
from geometry_msgs.msg import Point, PointStamped, Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA

import argparse
import line_notify

import hsrb_interface
from hsrb_interface import geometry
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

from jsk_recognition_msgs.msg import LabelArray, BoundingBoxArray, ClassificationResult, PeoplePoseArray, Accuracy
from hark_msgs.msg import HarkSource, HarkWave
from jsk_hsr_startup.robot_action import RobotAction
from jsk_hsr_startup.spot_mixin import SpotMixin
from jsk_hsr_startup.tmc_speak import speak_jp
from geometry_msgs.msg import PolygonStamped, PointStamped, PoseArray
from std_msgs.msg import Header
from my_robot import MyRobot

from sensor_msgs.msg import Image
from PIL import Image as _Image
from cv_bridge import CvBridge

class RecognizeSoundPos(MyRobot):
    def __init__(self, name):
        super(RecognizeSoundPos, self).__init__(name)
        self.whole_body = self.robot.get("whole_body")
        self.omni_base = self.robot.get("omni_base")
        rospack = rospkg.RosPack()
        self.client = line_notify.LineNotify(token="PuanmDsKMVCqqXdhpkh7MFth9qxsbhfRPTuBpZHGUvH")
        self.client.notify("init")

        self.flag = False
        self.ins_buffer = np.array([])
        self.time = Header()
        self.time2 = Header()
        self.threshold = rospy.get_param("~threshold", 1.2)
        self.prob_threshold = rospy.get_param("~prob_threshold", 0.05)
        self.bridge = CvBridge()
        self.sound_class_label = np.array([])
        self.save_dir = osp.join(rospack.get_path(
            "action_pkg"), "img_data")
        self.collision()

    def collision(self):
        collision_world.add_box(x=0.5, y=4.0, z=0.86, pose=geometry.pose(x=3.9, y=0.0, z=0.43), frame_id="map")
        self.whole_body.collision_world = collsion_world
    ####
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

        if np.all(self.ins_buffer[0:8] >= self.threshold):
            self.time = ins_msg.header
            self.flag = True
    ####
    
    ####
    def subscribe(self):
        sound_direction = message_filters.Subscriber("/wavdata_node_copy/max", HarkSource, queue_size=1)
        sound_class = message_filters.Subscriber("/sound_classifier/output", ClassificationResult, queue_size=1)
        accuracy = message_filters.Subscriber("/visualize_sound_max/prob", Accuracy, queue_size=1)
        use_async = rospy.get_param("~approximate_sync", True)
        queue_size = rospy.get_param("~queue_size", 1)
        self.subs = [sound_direction, sound_class, accuracy]
        if use_async:
            slop = rospy.get_param("~slop", 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                self.subs, queue_size, slop)
        else:
            sync = message_filters.TimeSynchronizer(self.subs, queue_size)
        sync.registerCallback(self.callback)
        
        #self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)
    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def callback(self, sd_msg, sc_msg, ac_msg):
        self.time2 = sd_msg.header
        print(self.time2.stamp.to_sec())

        max_direction = sd_msg.src[0]
        max_point, max_azimuth, max_elevation = self.dir_to_point(max_direction)
        print("max_point:", max_point)
        self.max_point = geometry.vector3(max_point[0], max_point[1], max_point[2])
        self.max_azimuth = max_azimuth

        #map_point = PointStamped()
        #map_point.header.frame_id = "map"

        camera_point = PointStamped()
        camera_mic_point = PointStamped()

        camera_point.header = sd_msg.header
        camera_point.header.frame_id = "head_rgbd_sensor_rgb_frame"
        camera_mic_point.header = sd_msg.header
        camera_mic_point.header.frame_id = "head_rgbd_sensor_rgb_frame"
        
        camera_to_mic_coords = self.get_a_to_b("head_rgbd_sensor_rgb_frame", "tamago1")
        camera_point.point.x, camera_point.point.y, camera_point.point.z = camera_to_mic_coords.transform_vector(max_point)
        camera_mic_point.point.x, camera_mic_point.point.y, camera_point.point.z = camera_to_mic_coords.transform_vector(np.array([0,0,0]))


        #p
        self.camera_mic_vector = np.array([camera_mic_point.point.x,
                                           camera_mic_point.point.y,
                                           camera_mic_point.point.z])
        #v
        self.direction_vector = np.array([camera_point.point.x - camera_mic_point.point.x,
                                          camera_point.point.y - camera_mic_point.point.y,
                                          camera_point.point.z - camera_mic_point.point.z])
        self.sound_class_label = np.append(self.sound_class_label ,sc_msg.label_names[0])
        self.sound_class_label = self.sound_class_label[-3:]
        self.accuracy = ac_msg.accuracy
    ####

    #subscribe_camera
    def subscribe_camera(self):
        sub_camera = rospy.Subscriber("/remote/head_rgbd_sensor/rgb/image_rect_color", Image, self.callback_camera, queue_size=1)

    def callback_camera(self, img_msg):
        self.color = self.bridge.imgmsg_to_cv2(img_msg, "rgb8")

    #subscribe box
    def subscribe_box(self):
        label_array = message_filters.Subscriber(
            "/mask_rcnn_instance_segmentation/output/labels", LabelArray)
        bbox_array = message_filters.Subscriber(
            "/multi_euclidean_cluster_point_indices_decomposer/boxes", BoundingBoxArray)
        pose_array = message_filters.Subscriber(
            "/multi_euclidean_cluster_point_indices_decomposer/centroid_pose_array", PoseArray)
        self.subs_box = [label_array, bbox_array, pose_array]
        if self.use_async:
            sync = message_filters.ApproximateTimeSynchronizer(self.subs_box, queue_size=1, slop=0.1)
        else:
            sync = message_filters.TimeSynchronizer(self.subs_box, queue_size=1)
        sync.registerCallback(self.box_callback)

    def unsubscribe_box(self):
        for sub in self.subs_box:
            sub.unregister()

    def box_callback(self, labels, boxes, poses):
        target_boxes = []
        grasp_hand_size = 0.08
        for label, box, pose in zip(labels.labels, boxes.boxes, poses.poses):
            print(label)
            if box.dimensions.y < grasp_hand_size:
                target_boxes.append((label, box, pose))
        self.target_boxes = target_boxes
        print("num:", len(self.target_boxes))

    ####
    def gaze_point(self):
        max_point_list = []
        max_azimuth_list = []
        self.subscribe()
        #self.subscribe_in_sound()
        self.subscribe_camera()
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while not self.flag:
            rate.sleep()
            print(self.sound_class_label)
            if all([e == "key\n" for e in self.sound_class_label]) and len(self.sound_class_label)==3:
                rospy.sleep(1.0)
                max_point_list.append(self.max_point)
                max_azimuth_list.append(self.max_azimuth)
                self.flag = True
            if (rospy.Time.now() - start_time).to_sec() > 30.0:
                return True
        #self.unsubscribe_in_sound()

        print("time:", self.time.stamp.to_sec())
        print("now:", rospy.Time.now().to_sec())

        # while self.time.stamp.to_sec() + 1.0 > rospy.Time.now().to_sec():
        #     rate.sleep()
        #     max_point_list.append(self.max_point)
        #     #direction_vector_list.append(self.direction_vector)
        #     if (rospy.Time.now() - start_time).to_sec() > 20.0:
        #         return True
        self.unsubscribe()

        print(max_point_list)
        self.client.notify("{} is ringing.".format(self.sound_class_label[0]))
        print("move around")

        if max_point_list[-1].z > 0.1:
            #self.whole_body.move_to_joint_positions({"arm_lift_joint": 0.4})
            self.whole_body.move_to_joint_positions({"arm_lift_joint": 0.4, "arm_flex_joint": -0.7})
        self.whole_body.gaze_point(point=max_point_list[-1], ref_frame_id="tamago1")

        rospy.sleep(1.0)

        print("accuracy:", self.accuracy)
        if self.accuracy < self.prob_threshold:
            self.client.notify("普段と違う場所にある可能性があります。")
        img_file = osp.join(self.save_dir, "{}_0000.jpg".format(self.sound_class_label[0]))
        cv2.imwrite(img_file, cv2.cvtColor(self.color.astype(np.uint8), cv2.COLOR_BGR2RGB))
        print("save camera")
        
        pil_img = _Image.open(img_file)
        self.client.notify(imgs=pil_img)

        #sound_direc_pose = geometry.pose(ek= max_azimuth_list[-1])
        #self.omni_base.go_pose(pose=sound_direc_pose, ref_frame_id="tamago1")

        # while True:
        #     retry = False
        #     self.subscribe_box()
        #     rate = rospy.Rate(10)
        #     start_time = rospy.Time.now()
        #     while len(self.target_boxes) == 0:
        #         print("now_num:", len(self.target_boxes))
        #         rate.sleep()
        #         if (rospy.Time.now() - start_time).to_sec() > 10.0:
        #             print("I couldn't find objects")
        #             return True
        #     self.unsubscribe_box()

        #     try:
        #         #self.omni_base.go_rel(x=0.1, y=0, yaw=0)
        #         self.whole_body.linear_weight = 100
        #         self.whole_body.angular_weight = 100

        #     except hsrb_interface.exceptions.GripperError:
        #         retry = True
        #     except hsrb_interface.exceptions.FollowTrajectoryError:
        #         retry = True
        #     except hsrb_interface.exceptions.MotionPlanningError:
        #         retry = True
        #     if retry:
        #         continue
        #     break
            
if __name__=="__main__":
    rospy.init_node("recognize_sound_pos")
    recognize_sound_pos = RecognizeSoundPos("a")
    recognize_sound_pos.gaze_point()
    

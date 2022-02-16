#!/usr/bin/env python
# -*- coding: utf-8 -*-

from cameramodels import PinholeCameraModel
import cv2
import math
import sys
import actionlib

from hsrb_interface import settings
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

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

import hsrb_interface
from hsrb_interface import geometry
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

from jsk_recognition_msgs.msg import LabelArray, BoundingBoxArray, ClassificationResult, PeoplePoseArray, Accuracy, RectArray, PolygonArray
from image_view2.msg import ImageMarker2
from hark_msgs.msg import HarkSource, HarkWave

from geometry_msgs.msg import PolygonStamped, PointStamped, PoseArray
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from my_robot import MyRobot

from PIL import Image as _Image
from cv_bridge import CvBridge, CvBridgeError

import skrobot
from skrobot.coordinates.math import quaternion2matrix, matrix2quaternion

class CloseFridgeDoor(MyRobot):
    def __init__(self, name, **kwargs):
        super(CloseFridgeDoor, self).__init__(name, **kwargs)
        self.move_base = actionlib.SimpleActionClient(
            "/move_base/move", MoveBaseAction)
        self.move_base.wait_for_server()

        settings._SETTINGS[u'trajectory']['action_timeout'] = 1
        self.whole_body = self.robot.get("whole_body")
        settings._SETTINGS[u'trajectory']['action_timeout'] = 30.0
        self.omni_base = self.robot.get("omni_base")
        self.gripper = self.robot.try_get('gripper')

        rospack = rospkg.RosPack()
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.bridge = CvBridge()

        self.camera_info_msg = rospy.get_param(
            "~camera_info", "/remote/head_rgbd_sensor/depth_registered/camera_info")
        self.camera_info = rospy.wait_for_message(
            self.camera_info_msg, CameraInfo)
        self.camera_model = PinholeCameraModel.from_camera_info(self.camera_info)
        print("finish init")

        self.sound_class = None
        self.sound_direction = None

        self.sound_classes = []
        self.sound_directions = []

    def subscribe_ssls(self):
        self.sub_ssls = rospy.Subscriber("/online/output_max_cls", ClassificationResult, self.ssls_callback, queue_size=1)
        #self.sub_ssls = rospy.Subscriber("/online/output_max_cls_kettle", ClassificationResult, self.ssls_callback, queue_size=1)

    def unsubscribe_ssls(self):
        self.sub_ssls.unregister()

    def ssls_callback(self, ssls_msg):
        self.sound_class = ssls_msg.label_names[0]
        self.sound_direction = ssls_msg.label_proba[0]

        self.sound_classes.append(self.sound_class)
        self.sound_directions.append(self.sound_direction)

        self.sound_classes = self.sound_classes[-10:]
        self.sound_directions = self.sound_directions[-10:]
        #print(max_point)

    # def subscribe_camera(self):
    #     sub_camera = rospy.Subscriber("/remote/head_rgbd_sensor/rgb/image_rect_color", Image, self.callback_camera, queue_size=1)

    # def callback_camera(self, img_msg):
    #     self.color = self.bridge.imgmsg_to_cv2(img_msg, "rgb8")

    def subscribe_box(self):
        label_array = message_filters.Subscriber(
            "/mask_rcnn_instance_segmentation_fridge/output/labels", LabelArray)
        bbox_array = message_filters.Subscriber(
            "/multi_euclidean_cluster_point_indices_decomposer_fridge/boxes", BoundingBoxArray)
        pose_array = message_filters.Subscriber(
            "/multi_euclidean_cluster_point_indices_decomposer_fridge/centroid_pose_array", PoseArray)
        rect_array = message_filters.Subscriber(
            "/mask_rcnn_instance_segmentation_fridge/output/rects", RectArray)
        self.subs_box = [label_array, bbox_array, pose_array, rect_array]

        use_async = True
        if use_async:
            sync = message_filters.ApproximateTimeSynchronizer(self.subs_box, queue_size=10, slop=0.2)
        else:
            sync = message_filters.TimeSynchronizer(self.subs_box, queue_size=10)
        sync.registerCallback(self.box_callback)

    def unsubscribe_box(self):
        for sub in self.subs_box:
            sub.unregister()

    def box_callback(self, labels, boxes, poses, rects):
        # print(len(rects.points))
        # rects_num = len(rects.points) // 8
        # x = 0
        # y = 0
        # for i in range(rects_num):
        #     for j in range(8):
        #         x += rects.points[8*i + j].x
        #         y += rects.points[8*i + j].y
        # print(x, y)
        xx = np.array([])
        yy = np.array([])
        zz = np.array([])
        for i in range(len(poses.poses)):
            x = poses.poses[i].position.x
            y = poses.poses[i].position.y
            z = poses.poses[i].position.z
            xx = np.append(xx,x)
            yy = np.append(yy,y)
            zz = np.append(zz,z)
            
        # uv, indices = self.camera_model.batch_project3d_to_pixel(
        #     np.concatenate([xx.reshape(-1,1),
        #                     yy.reshape(-1,1),
        #                     zz.reshape(-1,1)], axis=1),
        #     project_valid_depth_only=True, return_indices=True)
        # print(uv)

        # rcnn_center = [rects.rects[0].x + rects.rects[0].width/2, rects.rects[0].y + rects.rects[0].height/2]
        # print(rcnn_center)
        
        # min_distance = 10000
        # for i in range(len(uv)):
        #     distance = ((rcnn_center[0] - uv[i][0]) ** 2 + (rcnn_center[1] - uv[i][1]) ** 2)
        #     if distance < min_distance:
        #         min_distance = distance
        #         min_idx = i
        #print("min_idx:", min_idx)

        target_boxes = []
        grasp_hand_size = 0.08

        for i, (box, pose) in enumerate(zip(boxes.boxes, poses.poses)):
            #if box.dimensions.y < grasp_hand_size:
            #if label.name == "fridge" and i == min_idx:
            #if i == min_idx:
            target_boxes.append((box, pose))
        self.target_boxes = target_boxes
        print("num:", len(self.target_boxes))

    def subscribe_box2(self):
        label_array = message_filters.Subscriber(
            "/qatm/output/labels", LabelArray)
        #bbox_array = message_filters.Subscriber(
        #    "/multi_euclidean_cluster_point_indices_decomposer/boxes", BoundingBoxArray)
        #pose_array = message_filters.Subscriber(
        #    "/multi_euclidean_cluster_point_indices_decomposer/centroid_pose_array", PoseArray)
        rect_array = message_filters.Subscriber(
            "/qatm/output/rects", RectArray)
        depth = message_filters.Subscriber("/remote/head_rgbd_sensor/depth_registered/image", Image)
        planes = message_filters.Subscriber("/organized_multi_plane_segmentation/output_refined_polygon", PolygonArray)
        self.subs_box2 = [label_array, rect_array, depth, planes]
        use_async = True
        if use_async:
            sync = message_filters.ApproximateTimeSynchronizer(self.subs_box2, queue_size=100, slop=0.3)
        else:
            sync = message_filters.TimeSynchronizer(self.subs_box2, queue_size=100)
        sync.registerCallback(self.box_callback2)

    def unsubscribe_box2(self):
        for sub in self.subs_box2:
            sub.unregister()

    def box_callback2(self, labels, rects, depth, planes):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth, "passthrough")
        except CvBridgeError, e:
            rospy.logerr(e)

        # xx = np.array([])
        # yy = np.array([])
        # zz = np.array([])
        # for i in range(len(poses.poses)):
        #     x = poses.poses[i].position.x
        #     y = poses.poses[i].position.y
        #     z = poses.poses[i].position.z
        #     xx = np.append(xx,x)
        #     yy = np.append(yy,y)
        #     zz = np.append(zz,z)

        # uv, indices = self.camera_model.batch_project3d_to_pixel(
        #     np.concatenate([xx.reshape(-1,1),
        #                     yy.reshape(-1,1),
        #                     zz.reshape(-1,1)], axis=1),
        #     project_valid_depth_only=True, return_indices=True)
        #print(uv)

        labels_list = []
        tool_idxs = []
        button_idxs = []
        for i in range(len(labels.labels)):
            labels_list.append(labels.labels[i].name)
            #print(i)
            if labels.labels[i].name == "tool":
                tool_idxs.append(i)
            elif labels.labels[i].name == "stop":
                button_idxs.append(i)

        self.tool_poses = []
        self.button_poses = []
        self.plane_poses = []
        for tool_idx in tool_idxs:
            rect_x = np.arange(rects.rects[tool_idx].x, rects.rects[tool_idx].x + rects.rects[tool_idx].width)
            rect_y = np.arange(rects.rects[tool_idx].y, rects.rects[tool_idx].y + rects.rects[tool_idx].height)
            aa, bb = np.meshgrid(rect_x, rect_y)
            uv = np.array((aa.reshape(-1), bb.reshape(-1))).T
            
            tool_center = np.array([rects.rects[tool_idx].x + rects.rects[tool_idx].width/2, rects.rects[tool_idx].y + rects.rects[tool_idx].height/2])
            #button_center = [rects.rects[button_idx].x + rects.rects[button_idx].width/2, rects.rects[button_idx].y + rects.rects[button_idx].height/2]

            #print(uv.shape)
            cropped_depth = depth_image[rects.rects[tool_idx].y : rects.rects[tool_idx].y + rects.rects[tool_idx].height , rects.rects[tool_idx].x : rects.rects[tool_idx].x + rects.rects[tool_idx].width]
            cropped_center_depth = depth_image[rects.rects[tool_idx].y + rects.rects[tool_idx].height/2, rects.rects[tool_idx].x + rects.rects[tool_idx].width/2]
            #print(cropped_depth.shape)
            xyz = self.camera_model.batch_project_pixel_to_3d_ray(uv, cropped_depth)
            center_xyz = self.camera_model.project_pixel_to_3d_ray(tool_center, cropped_center_depth)

            self.tool_pose = np.nanmean(xyz, axis=0)
            #self.tool_pose = np.array(center_xyz)
            self.tool_poses.append(self.tool_pose)

        for button_idx in button_idxs:
            rect_x = np.arange(rects.rects[button_idx].x, rects.rects[button_idx].x + rects.rects[button_idx].width)
            rect_y = np.arange(rects.rects[button_idx].y, rects.rects[button_idx].y + rects.rects[button_idx].height)
            aa, bb = np.meshgrid(rect_x, rect_y)
            uv = np.array((aa.reshape(-1), bb.reshape(-1))).T
            
            button_center = np.array([rects.rects[button_idx].x + rects.rects[button_idx].width/2, rects.rects[button_idx].y + rects.rects[button_idx].height/2])
            #print(uv.shape)
            cropped_depth = depth_image[rects.rects[button_idx].y : rects.rects[button_idx].y + rects.rects[button_idx].height , rects.rects[button_idx].x : rects.rects[button_idx].x + rects.rects[button_idx].width]
            cropped_center_depth = depth_image[rects.rects[button_idx].y + rects.rects[button_idx].height/2, rects.rects[button_idx].x + rects.rects[button_idx].width/2]
            #print(cropped_depth.shape)
            xyz = self.camera_model.batch_project_pixel_to_3d_ray(uv, cropped_depth)
            center_xyz = self.camera_model.project_pixel_to_3d_ray(tool_center, cropped_center_depth)

            self.button_pose = np.nanmean(xyz, axis=0)
            #self.tool_pose = np.array(center_xyz)
            self.button_poses.append(self.button_pose)
        #print(self.tool_poses)
        #print(self.button_poses)

        for plane_idx in range(len(planes.polygons)):
            self.plane_points = planes.polygons[plane_idx].polygon.points
            #print(self.plane_points)

            plane_x = 0
            plane_y = 0
            plane_z = 0
            for i in range(len(self.plane_points)):
                plane_x += self.plane_points[i].x
                plane_y += self.plane_points[i].y
                plane_z += self.plane_points[i].z
            plane_x /= len(self.plane_points)
            plane_y /= len(self.plane_points)
            plane_z /= len(self.plane_points)

            self.plane_pose = np.array([plane_x, plane_y, plane_z])
            self.plane_poses.append(self.plane_pose)
        print(self.plane_poses)

    def stop_button2(self):
        self.whole_body.move_to_neutral()
        while True:
            act = self.move_to('/eng8/6f/room610-kettle-front', wait=True)
            self.whole_body.move_to_neutral()
            #self.omni_base.go_rel(0.0, -0.1, 0.0, 100.0)
            self.whole_body.move_to_joint_positions({"arm_lift_joint": 0.2})
            self.whole_body.move_to_joint_positions({"head_tilt_joint": 0.0})
            self.whole_body.move_to_joint_positions({"arm_roll_joint": 2.2})
            self.whole_body.move_to_joint_positions({"head_tilt_joint": -0.2})
            rospy.sleep(10.0)
            #     #push
            print("push mode")
            self.button_poses = []
            self.subscribe_box2()
            rate = rospy.Rate(10)
            start_time = rospy.Time.now()
            while len(self.button_poses) == 0:
                rate.sleep()
                if (rospy.Time.now() - start_time).to_sec() > 15.0:
                    break
            self.unsubscribe_box2()
            retry = False
            self.button_poses = sorted(self.button_poses, key=lambda a: a[0], reverse=True)
            print("button_poses:",self.button_poses)
            self.button_pose = self.button_poses[0]

            self.plane_poses = sorted(self.plane_poses, key=lambda a: a[2])
            print("plane_poses:", self.plane_poses)
            plane_pos = self.plane_poses[0]
            try:
                self.whole_body.linear_weight = 100
                self.whole_body.angular_weight = 100

                base_to_camera_coords = self.get_a_to_b("/base_link", "/head_rgbd_sensor_rgb_frame")
                plane_pos_x, plane_pos_y, plane_pos_z = base_to_camera_coords.transform_vector(
                    plane_pos)
                print("plane_pos")
                print(plane_pos_x, plane_pos_y, plane_pos_z)
                pos_x, pos_y, pos_z = base_to_camera_coords.transform_vector(
                    self.button_pose)

                rot = np.matrix(quaternion2matrix((1,0,0,0)))
                button_c = skrobot.coordinates.Coordinates(
                    pos=(pos_x, pos_y, 0.978), rot=rot)
                button_c.rotate(np.pi, 'y')
                button_c.rotate(np.pi, 'z')
                button_c.rotate(np.pi/15.0, 'y')

                pre_push_coords = button_c.copy_worldcoords()
                pre_push_coords.translate(np.array([0.0, 0, 0]), 'local')

                q_wxyz = matrix2quaternion(pre_push_coords.worldrot())
                self.whole_body.move_to_joint_positions({"arm_lift_joint": 0.5})
                self.whole_body.move_to_joint_positions({"arm_roll_joint": 0.0})
                self.gripper.apply_force(1.0)
                
                self.whole_body.move_end_effector_pose(
                    geometry.Pose(
                        pos=geometry.Vector3(
                            pre_push_coords.worldpos()[0],
                            pre_push_coords.worldpos()[1],
                            pre_push_coords.worldpos()[2]),
                        ori=geometry.Quaternion(
                            q_wxyz[1],
                            q_wxyz[2],
                            q_wxyz[3],
                            q_wxyz[0])),
                    ref_frame_id='base_link')

                rospy.sleep(10.0)
                #self.whole_body.impedance_config = "compliance_hard"
                current_end_effector_pose = self.whole_body.get_end_effector_pose()
                while current_end_effector_pose.pos.z > 0.947:
                    self.whole_body.move_end_effector_pose(
                        geometry.pose(z=0.01), ref_frame_id="hand_palm_link")
                    current_end_effector_pose = self.whole_body.get_end_effector_pose()

                self.whole_body.move_end_effector_pose(
                    geometry.pose(z=-0.1), ref_frame_id="hand_palm_link")
                self.gripper.command(0.5)
                #初期位置に元の場所に戻る
                self.move_to('/eng8/6f/room610-kettle-front', wait=True)
                self.whole_body.move_to_neutral()
            except hsrb_interface.exceptions.GripperError:
                print("gripper error")
                #speak_jp('グリッパーエラーです。')
                retry = True
            except hsrb_interface.exceptions.FollowTrajectoryError:
                print("follw trajectory error")
                #speak_jp('フォロージョイントトラジェクトリーのエラーです。')
                retry = True
            if retry:
                continue
            break
        
    def stop_button(self):
        self.whole_body.move_to_neutral()
        while True:
            act = self.move_to('/eng8/6f/room610-kettle-front', wait=True)
            self.whole_body.move_to_neutral()
            #self.omni_base.go_rel(0.0, -0.1, 0.0, 100.0)

            self.whole_body.move_to_joint_positions({"arm_lift_joint": 0.2})
            self.whole_body.move_to_joint_positions({"head_tilt_joint": 0.0})
            rospy.sleep(5.0)

            self.tool_poses = []
            self.button_poses = []
            self.subscribe_box2()
            rate = rospy.Rate(10)
            start_time = rospy.Time.now()
            while len(self.tool_poses) == 0:
                rate.sleep()
                if (rospy.Time.now() - start_time).to_sec() > 15.0:
                    break
            self.unsubscribe_box2()
            retry = False

            self.tool_poses = sorted(self.tool_poses, key=lambda a: a[2])
            #print(self.tool_poses)
            self.tool_pose = self.tool_poses[0]

            self.plane_poses = sorted(self.plane_poses, key=lambda a: a[2])
            plane_pos = self.plane_poses[0]
            
            try:
                self.whole_body.linear_weight = 100
                self.whole_body.angular_weight = 100
                rot = np.matrix(quaternion2matrix((1,0,0,0)))
                pos = self.tool_pose
                base_to_camera_coords = self.get_a_to_b("/base_link", "/head_rgbd_sensor_rgb_frame")
                plane_pos_x, plane_pos_y, plane_pos_z = base_to_camera_coords.transform_vector(
                    plane_pos)
                pos_x, pos_y, pos_z = base_to_camera_coords.transform_vector(
                    pos)
                # print(pos_x, pos_y, pos_z)
                c = skrobot.coordinates.Coordinates(
                    pos=(plane_pos_x, pos_y, pos_z), rot=rot)
                c.rotate(np.pi / 2.0, 'y')
                pre_grasp_coords = c.copy_worldcoords()
                pre_z = -0.10
                pre_grasp_coords.translate(np.array([0, 0, pre_z]), 'local')

                q_wxyz = matrix2quaternion(pre_grasp_coords.worldrot())
                self.whole_body.move_end_effector_pose(
                    geometry.Pose(
                        pos=geometry.Vector3(
                            pre_grasp_coords.worldpos()[0],
                            pre_grasp_coords.worldpos()[1],
                            pre_grasp_coords.worldpos()[2]),
                        ori=geometry.Quaternion(
                            q_wxyz[1],
                            q_wxyz[2],
                            q_wxyz[3],
                            q_wxyz[0])),
                    ref_frame_id='base_link')
                self.gripper.command(0.5)

                self.whole_body.move_end_effector_pose(
                    geometry.Pose(
                        pos=geometry.Vector3(0, 0, 0.05),
                        ori=geometry.Quaternion(0, 0, 0, 1)),
                    ref_frame_id='hand_palm_link')
                self.gripper.apply_force(0.1)

                distance = self.gripper.get_distance()
                rospy.loginfo('gripper distance {}'.format(distance))
                if distance < 0.013:
                    #catch failed
                    retry = True
                    self.gripper.command(0.5)
                    continue
                else:
                    self.gripper.apply_force(1.0)
                print("catch succeed")
                ### catch succeed ###
                self.whole_body.move_end_effector_pose(
                    geometry.pose(x=-0.1),
                    ref_frame_id="hand_palm_link")

            except hsrb_interface.exceptions.GripperError:
                print("gripper error")
                #speak_jp('グリッパーエラーです。')
                retry = True
            except hsrb_interface.exceptions.FollowTrajectoryError:
                print("follw trajectory error")
                #speak_jp('フォロージョイントトラジェクトリーのエラーです。')
                retry = True
            
            if retry:
                continue
            break

        while True:
            act = self.move_to('/eng8/6f/room610-kettle-front', wait=True)
            self.whole_body.move_to_neutral()
            #self.omni_base.go_rel(0.0, -0.1, 0.0, 100.0)

            self.whole_body.move_to_joint_positions({"arm_lift_joint": 0.2})
            self.whole_body.move_to_joint_positions({"head_tilt_joint": 0.0})
            self.whole_body.move_to_joint_positions({"arm_roll_joint": 2.2})
            self.whole_body.move_to_joint_positions({"head_tilt_joint": -0.2})
            rospy.sleep(10.0)

            #     #push
            print("push mode")
            self.subscribe_box2()
            rate = rospy.Rate(10)
            start_time = rospy.Time.now()
            while len(self.button_poses) == 0:
                rate.sleep()
                if (rospy.Time.now() - start_time).to_sec() > 15.0:
                    break
            self.unsubscribe_box2()
            retry = False
            self.button_poses = sorted(self.button_poses, key=lambda a: a[0], reverse=True)
            print("button_poses:",self.button_poses)
            self.button_pose = self.button_poses[0]

            self.plane_poses = sorted(self.plane_poses, key=lambda a: a[2])
            print("plane_poses:", self.plane_poses)
            plane_pos = self.plane_poses[0]
            try:
                base_to_camera_coords = self.get_a_to_b("/base_link", "/head_rgbd_sensor_rgb_frame")
                plane_pos_x, plane_pos_y, plane_pos_z = base_to_camera_coords.transform_vector(
                    plane_pos)
                print("plane_pos")
                print(plane_pos_x, plane_pos_y, plane_pos_z)
                pos_x, pos_y, pos_z = base_to_camera_coords.transform_vector(
                    self.button_pose)

                button_c = skrobot.coordinates.Coordinates(
                    pos=(plane_pos_x+0.02, pos_y, 0.978), rot=rot)
                button_c.rotate(np.pi / 2.0, 'y')
                button_c.rotate(np.pi, 'z')

                pre_push_coords = button_c.copy_worldcoords()
                pre_push_coords.translate(np.array([0.0, 0, 0]), 'local')

                q_wxyz = matrix2quaternion(pre_push_coords.worldrot())
                self.whole_body.move_end_effector_pose(
                    geometry.Pose(
                        pos=geometry.Vector3(
                            pre_push_coords.worldpos()[0],
                            pre_push_coords.worldpos()[1],
                            pre_push_coords.worldpos()[2]),
                        ori=geometry.Quaternion(
                            q_wxyz[1],
                            q_wxyz[2],
                            q_wxyz[3],
                            q_wxyz[0])),
                    ref_frame_id='base_link')

                #self.whole_body.impedance_config = "compliance_hard"
                current_end_effector_pose = self.whole_body.get_end_effector_pose()
                while current_end_effector_pose.pos.z > 0.930:
                    self.whole_body.move_end_effector_pose(
                        geometry.pose(x=-0.01), ref_frame_id="hand_palm_link")
                    current_end_effector_pose = self.whole_body.get_end_effector_pose()
                    
                #tool を離す
                self.whole_body.move_end_effector_pose(
                    geometry.pose(x=0.1), ref_frame_id="hand_palm_link")
                self.whole_body.move_end_effector_pose(
                    geometry.pose(x=-0.1, y=0.1, ek=3.14), ref_frame_id="hand_palm_link")
                self.gripper.command(0.5)
                
                #初期位置に元の場所に戻る
                self.move_to('/eng8/6f/room610-kettle-front', wait=True)
                self.whole_body.move_to_neutral()
            except hsrb_interface.exceptions.GripperError:
                print("gripper error")
                #speak_jp('グリッパーエラーです。')
                retry = True
            except hsrb_interface.exceptions.FollowTrajectoryError:
                print("follw trajectory error")
                #speak_jp('フォロージョイントトラジェクトリーのエラーです。')
                retry = True
            if retry:
                continue
            break
        
    def close_door(self):
        while True:
            act = self.move_to('/eng8/6f/room610-fridge-front', wait=True)
            self.whole_body.move_to_neutral()

            self.whole_body.move_to_joint_positions({"arm_lift_joint": 0.2})
            rospy.sleep(7.0)
            
            self.target_boxes = []
            self.subscribe_box()
            rate = rospy.Rate(10)
            start_time = rospy.Time.now()
            while len(self.target_boxes) == 0:
                rate.sleep()
                if (rospy.Time.now() - start_time).to_sec() > 10.0:
                    return True
            self.unsubscribe_box()
            retry = False
            print(self.target_boxes)

            target_pose = self.target_boxes[0][0].pose
            try:
                self.whole_body.linear_weight = 100
                self.whole_body.angular_weight = 100
                rot = np.matrix(quaternion2matrix((target_pose.orientation.w,
                                                   target_pose.orientation.x,
                                                   target_pose.orientation.y,
                                                   target_pose.orientation.z)))
                pos = np.array([target_pose.position.x,
                                target_pose.position.y,
                                target_pose.position.z])

                c = skrobot.coordinates.Coordinates(
                    pos=pos, rot=rot)

                #c.rotate(np.pi / 2.0, "y")
                #if target_pose.orientation.x < 0:
                #    c.rotate(np.pi, 'z')
                #    print("rotate coords")

                #c.rotate(np.pi/2, "y")
                #c.rotate(np.pi, "z")

                print(rot)
                if rot[0,0] > 0.66:
                    c.rotate(np.pi/2.0, "y")
                    print("aaaaaa")
                else:
                    c.rotate(-np.pi/2.0, "x")
                    print("bbbbb")
                pre_grasp_coords = c.copy_worldcoords()
                pre_z = -0.2
                pre_grasp_coords.translate(np.array([0, 0, pre_z]), 'local')

                q_wxyz = matrix2quaternion(pre_grasp_coords.worldrot())
                self.whole_body.move_end_effector_pose(
                    geometry.Pose(
                        pos=geometry.Vector3(
                            pre_grasp_coords.worldpos()[0],
                            pre_grasp_coords.worldpos()[1],
                            pre_grasp_coords.worldpos()[2]),
                        ori=geometry.Quaternion(
                            q_wxyz[1],
                            q_wxyz[2],
                            q_wxyz[3],
                            q_wxyz[0])),
                    ref_frame_id='base_link')
                self.gripper.command(0.5)

                self.gripper.apply_force(0.1)
                self.whole_body.move_end_effector_pose(
                    geometry.Pose(
                        pos=geometry.Vector3(0, 0, -pre_z-0.05),
                        ori=geometry.Quaternion(0, 0, 0, 1)),
                    ref_frame_id='hand_palm_link')


                # self.whole_body.move_end_effector_by_arc(
                #     geometry.pose(y=0.25, z=0.02, ej=math.radians(90.0)),
                #     math.radians(-20.0),
                #     ref_frame_id="hand_palm_link")

                self.whole_body.move_to_neutral()
                self.gripper.command(0.5)

            except hsrb_interface.exceptions.GripperError:
                print("gripper error")
                #speak_jp('グリッパーエラーです。')
                retry = True
            except hsrb_interface.exceptions.FollowTrajectoryError:
                print("follw trajectory error")
                #speak_jp('フォロージョイントトラジェクトリーのエラーです。')
                retry = True

            if retry:
                print("continue")
                continue
            break

    def run(self):
        #act = self.move_to('/eng8/6f/room610-kettle-front', wait=True)
        #self.whole_body.move_to_neutral()
        #rospy.sleep(10.0)
        
        self.subscribe_ssls()
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        point_list = []
        while True:
            rate.sleep()
            print(self.sound_classes)
            print(self.sound_directions)
            if self.sound_classes.count("fridge") >= 5:
                print("ok")
                direction = -1
                for d in range(8):
                    if self.sound_directions.count(d*45) > 3:
                        direction = d * 45
                if direction == -1:
                    continue
                rospy.sleep(1.0)
                azimuth = np.radians(direction)
                elevation = np.radians(0)
                
                x = np.cos(elevation) * np.cos(azimuth)
                y = np.cos(elevation) * np.sin(azimuth)
                z = np.sin(elevation)
                point = np.array([x,y,z])
                
                geo_point = geometry.vector3(point[0], point[1], point[2])
                point_list.append(geo_point)
                break
            if (rospy.Time.now() - start_time).to_sec() > 15.0:
                break
        self.unsubscribe_ssls()

        if len(point_list) == 0:
            self.run()
            return
        self.whole_body.gaze_point(point=point_list[-1], ref_frame_id="tamago1")
        rospy.sleep(10.0)
        self.close_door()

    def run2(self):
        self.subscribe_ssls()
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        point_list = []
        while True:
            rate.sleep()
            print(self.sound_classes)
            print(self.sound_directions)
            if self.sound_classes.count("kettle") >= 5:
                print("ok")
                direction = -1
                for d in range(8):
                    if self.sound_directions.count(d*45) > 3:
                        direction = d * 45
                if direction == -1:
                    continue
                rospy.sleep(1.0)
                azimuth = np.radians(direction)
                elevation = np.radians(0)
                
                x = np.cos(elevation) * np.cos(azimuth)
                y = np.cos(elevation) * np.sin(azimuth)
                z = np.sin(elevation)
                point = np.array([x,y,z])
                
                geo_point = geometry.vector3(point[0], point[1], point[2])
                point_list.append(geo_point)
                break
            if (rospy.Time.now() - start_time).to_sec() > 15.0:
                break
        self.unsubscribe_ssls()
        self.whole_body.gaze_point(point=point_list[-1], ref_frame_id="tamago1")
        rospy.sleep(10.0)

        act = self.move_to('/eng8/6f/room610-kettle-front', wait=True)
    

if __name__ == "__main__":
    rospy.init_node("close_door_demo")
    act = CloseFridgeDoor("a")
    #act.close_door()
    #act.stop_button()
    
    #act.stop_button2()
    act.run()
    #act.run2()

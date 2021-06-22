#!/usr/bin/env python
# -*- coding: utf-8 -*-
import open3d as o3d
import cv2
import message_filters
import numpy as np
import os
import pathlib2
import rospy
import skrobot
import subprocess
import sys
import tf
import image_geometry
import time

from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool, SetBoolResponse
from audio_common_msgs.msg import AudioData
import wavio

class CameraSave():
    def __init__(self):
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.input_color = rospy.get_param(
            '~input_color',
            '/camera/rgb/image_rect_color')
        self.input_depth = rospy.get_param(
            '~input_depth',
            '/camera/depth_registered/sw_registered/image_rect_raw')
        # self.input_mask = rospy.get_param(
        #     '~input_mask',
        #     '/color_filter_cluster_decomposer/mask')
        self.input_audio = rospy.get_param(
            "~input_audio",
            "/audio")

        self.camera_info_msg = rospy.get_param(
            '~camera_info', '/camera/rgb/camera_info')

        self.save_raw_img = rospy.get_param(
            '~save_raw_img', True)

        self.save_dir = rospy.get_param(
            '~save_dir', 'save_dir/')

        self.save_dir = os.path.join(self.current_dir, '..', self.save_dir)
        pathlib2.Path(os.path.join(self.save_dir, 'raw')).mkdir(
            parents=True, exist_ok=True)
        pathlib2.Path(os.path.join(self.save_dir, 'camera_pose')).mkdir(
            parents=True, exist_ok=True)

        self.camera_info = None
        self.camera_model = image_geometry.cameramodels.PinholeCameraModel()
        self.color = None
        self.depth = None
        self.header = None
        self.stanby = False
        self.callback_lock = False

        self.data8 = None
        self.a_lis = np.empty((0,8))

        self.load_camera_info()
        self.subscribe_camera()
        self.subscribe_audio()
        self.bridge = CvBridge()

        #print("aa")
        self.lis = tf.TransformListener()
        self.count = 0
        # self.voxel_length = 0.002
        # self.volume = o3d.integration.ScalableTSDFVolume(
        #     voxel_length=0.0025,
        #     sdf_trunc=0.01,
        #     color_type=o3d.integration.TSDFVolumeColorType.RGB8)
        self.service()

    def load_camera_info(self):
        self.camera_info = rospy.wait_for_message(
            self.camera_info_msg, CameraInfo)
        self.camera_model.fromCameraInfo(self.camera_info)
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic()
        self.intrinsic.set_intrinsics(
            self.camera_model.width,
            self.camera_model.height,
            self.camera_model.fx(),
            self.camera_model.fy(),
            self.camera_model.cx(),
            self.camera_model.cy())
        print('load camera model')
        np.savetxt(os.path.join(self.save_dir, 'camera_pose/intrinsic.txt'),
                   self.intrinsic.intrinsic_matrix)

    def subscribe_camera(self):
        sub_color = message_filters.Subscriber(
            self.input_color, Image, queue_size=10)
        sub_depth = message_filters.Subscriber(
            self.input_depth, Image, queue_size=10)
        # sub_mask = message_filters.Subscriber(
        #     self.input_mask, Image, queue_size=10)
        #self.subs = [sub_color, sub_depth, sub_mask]
        self.subs = [sub_color, sub_depth]
        #self.subs = [sub_color, sub_depth]
        # sync = message_filters.TimeSynchronizer(
        #     fs=self.subs, queue_size=100)
        sync = message_filters.ApproximateTimeSynchronizer(self.subs, queue_size=100, slop=0.1)
        sync.registerCallback(self.callback_camera)


    def callback_camera(self, rgb_msg, depth_msg):
        if self.callback_lock:
            return
        #self.mask = self.bridge.imgmsg_to_cv2(mask_msg, 'mono8')
        self.color = self.bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')
        self.depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        self.header = rgb_msg.header

        #print(self.depth[100][100:110])
        #print(self.color.shape)
        if not self.stanby:
            rospy.loginfo('Stanby!')
            self.stanby = True

    def subscribe_audio(self):
        sub_audio = rospy.Subscriber(self.input_audio, AudioData, self.callback_audio, queue_size=1000, buff_size=2**24)

    def callback_audio(self, msg):
        if self.callback_lock:
            return
        data = msg.data
        data16 = np.frombuffer(data, dtype="int16")
        data16 = np.array(data16)
        data16 = np.where(data16 > 0, data16/(2.0**31 -1), data16/(2.0**31))
        self.data8 = data16.reshape(-1,8)
        self.a_lis = np.append(self.a_lis, self.data8, axis=0)
        self.a_lis = self.a_lis[-160000:]

    def service(self):
        #print("bbbbbbbbbbbbb")
        self.camera_service = rospy.Service("save_shots", SetBool, self.save_shots)
        # self.integrate_service = rospy.Service('integrate_point_cloud',
        #                                        SetBool,
        #                                        self.integrate_point_cloud)

        #self.audio_service = rospy.Service("save_audio", SetBool, self.save_audio)

    def save_shots(self, req):
        #self.callback_lock = True
        start = time.time()
        for i in range(160):
            rospy.sleep(1.0/16.0 - 0.015)
            #time.sleep(1.0/16.0 - 0.015)
            #self.color_clip = self.color.copy()
            #self.depth_clip = self.depth.copy()

            rospy.loginfo(rospy.get_time())
            rospy.loginfo("count : %d", self.count)

            try:
                cv2.imwrite(os.path.join(self.save_dir, "color{:03}.png".format(
                    self.count)), cv2.cvtColor(
                        self.color.astype(np.uint8), cv2.COLOR_BGR2RGB))
                print("zzzz")

                cv2.imwrite(os.path.join(self.save_dir, 'depth{:03}.png'.format(
                    self.count)), self.depth.astype(np.uint16))

                print("yyyyy")
                self.count += 1
                self.callback_lock = False
                #return SetBoolResponse(True, "success")

            except Exception:
                self.callback_lock = False
                return SetBoolResponse(False, 'failed')

        el_time = time.time() -start

        wavio.write(os.path.join(self.save_dir, "sound.wav"), self.a_lis, 16000, sampwidth=3)
        print("el_time:", el_time)

        return SetBoolResponse(True, "success")

    def run(self):
        try:
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                #rospy.loginfo("aaa")
                rate.sleep()
        except KeyboardInterrupt:
            sys.exit(0)

if __name__=="__main__":
    rospy.init_node("camera_save", anonymous=False)
    camera_save = CameraSave()
    camera_save.run()

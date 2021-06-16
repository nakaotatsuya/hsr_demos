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

import pyroomacoustics as pr
import scipy.signal as sig
from sklearn.decomposition import PCA
import glob
import scipy.io.wavfile as wav

#from jsk_topic_tools import ConnectionBasedTransport

class MicSave():
    def __init__(self):
        rospy.loginfo("init")
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        # self.input_audio = rospy.get_param(
        #     "~input_audio",
        #     "/audio")
        # self.in_sound = rospy.get_param(
        #     "~in_sound",
        #     "/sound_detector_volume/in_sound")
        self.save_dir = rospy.get_param(
            '~save_dir', 'save_dir/')
        self.save_dir = os.path.join(self.current_dir, '..', self.save_dir)
        if not os.path.exists(self.save_dir):
            os.mkdir(self.save_dir)
        #self.data8 = None
        self.a_lis = np.empty((0,8))
        self.count = 0
        self.service()
        self.callback_lock=False

        self.subscribe()

        #self.subscribe_audio()
        rospy.loginfo("init end")

    def subscribe(self):
        sub = rospy.Subscriber("~input", AudioData, self.callback_audio, queue_size=1000, buff_size=2**24)
        #sub_in_sound = rospy.Subscriber("~in_sound", InSound, self.callback_in_sound, queue_size=1)
        self.subs = [sub]
    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()
        
    # def subscribe_audio(self):
    #     sub_audio = rospy.Subscriber(self.input_audio, AudioData, self.callback_audio, queue_size=1000, buff_size=2**24)

    def callback_audio(self, msg):
        #print("in callback")
        #if self.callback_lock:
        #    return
        data = msg.data
        data16 = np.frombuffer(data, dtype="int16")
        data16 = np.array(data16)
        #data16 = np.where(data16 > 0, data16/(2.0**31 -1), data16/(2.0**31))
        data8 = data16.reshape(-1,8)
        self.a_lis = np.append(self.a_lis, data8, axis=0)
        self.a_lis = self.a_lis[-80000:]
        #print(self.a_lis.shape)

    # def subscribe_in_sound(self):
    #     self.sub_in_sound = rospy.Subscriber(self.in_sound, InSound, self.callback_in_sound, queue_size=1)

    # def unsubscribe(self):
    #     self.sub_in_sound.unregister()

    # def call_service(data):
    #     print("s")
    #     try:
    #         service = rospy.ServiceProxy("save_wave", SetBool)
    #         resp = service()
    #     except rospy.ServiceException, e:
    #         print "Service call failed: %s"%e

    def service(self):
        srv = rospy.Service("save_wave", SetBool, self.save_wave)

    def save_wave(self, req):
        #self.callback_lock = True
        print("save wave")
        try:
            start = time.time()
            for i in range(8):
                wavio.write(os.path.join(self.save_dir, "sound_{}.wav".format(i)), self.a_lis.T[i], 16000, sampwidth=3)

            self.ilrma()
            el_time = time.time() - start
            print("el_time:", el_time)
            #self.callback_lock=False
        except Exception:
            print("failed")
            return SetBoolResponse(False, "failed")
        return SetBoolResponse(True, "success")

    def ilrma(self):
        pca = PCA(n_components=2, whiten=True)
        H = pca.fit_transform(self.a_lis)
        n = 0.3*16000
        i = 1
        while(i*2 <= n):
            i *= 2
        seg = i*2
        stft_list = []
        for i in range(2):
            _,_,Z = sig.stft(H[:,i], nperseg = seg)
            stft_list.append(Z.T)
        f_data = np.stack(stft_list, axis=2)
        Array = pr.bss.ilrma(f_data, n_src=None, n_iter=100, proj_back=True, W0=None, n_components=2, return_filters=False, callback=None)
        sep = []
        for i in range(2):
            x = sig.istft(Array[:,:,-(i+1)].T, nperseg = seg)
            sep.append(x[1])
        #print(sep[0])
        #print(sep[1])
        wavio.write(os.path.join(self.save_dir, "separate_0.wav"), (sep[0]*100).astype(np.int16), 16000, sampwidth=3)
        wavio.write(os.path.join(self.save_dir, "separate_1.wav"), (sep[1]*100).astype(np.int16), 16000, sampwidth=3)

    def run(self):
        try:
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                #rospy.loginfo("aaa")
                rate.sleep()
        except KeyboardInterrupt:
            sys.exit(0)

if __name__=="__main__":
    rospy.init_node("mic_8", anonymous=True)
    mic_save = MicSave()
    mic_save.run()
    #rospy.spin()

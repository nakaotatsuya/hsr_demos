#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
from geometry_msgs.msg import Point, PointStamped, Point, Quaternion
from std_msgs.msg import ColorRGBA

import itertools
import sympy

import argparse

rospy.init_node("center")
rospack = rospkg.RosPack()

parser = argparse.ArgumentParser()
parser.add_argument("-l", "--label", type=str, default="key")
#parser.add_argument("-n", "--number", type=int, default=1)
args = parser.parse_args()

save_dir = osp.join(rospack.get_path(
    'action_pkg'), "sound_pos_data", args.label)

load_file = np.load(osp.join(save_dir, "center.npy"), allow_pickle=True)
print(load_file)

new_center = np.empty((0,3))
for i in range(load_file.shape[0]):
    if load_file[i,2] < 2.0 and load_file[i,2] > -1.0:
        #print(load_file[i])
        new_center = np.vstack((new_center, load_file[i]))

#print(new_center)
#np.delete(load_file, np.where(load_file[:,2] > 3.0)[0], axis=0)
np.save(osp.join(save_dir, "center.npy"), new_center)

# new_center = np.empty((0,3))
# for i in range(load_file.shape[0]):
#     if load_file[i,0] < 7.0:
#         new_center = np.vstack((new_center, load_file[i]))

np.save(osp.join(save_dir, "center.npy"), new_center)

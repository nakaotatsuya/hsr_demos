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

rospy.init_node("testest")
rospack = rospkg.RosPack()

parser = argparse.ArgumentParser()
parser.add_argument("-l", "--label", type=str, default="test")
parser.add_argument("-n", "--number", type=int, default=1)
args = parser.parse_args()

save_dir = osp.join(rospack.get_path(
    'action_pkg'), "sound_pos_data", args.label)

file_num = len(listdir(save_dir))

load_file = np.load(osp.join(save_dir, "{:0=5d}.npy").format(args.number))
print(load_file)

p = load_file[0]
v = load_file[1]
q = p + v

pub_msg = Marker()
pub_msg.header.stamp = rospy.Time.now()
pub_msg.header.frame_id = "map"
pub_msg.type = 5

pub_msg.scale.x = 0.1
pub_msg.scale.y = 0.1
pub_msg.scale.z = 0.1


pub_msg.points.append(Point(x=p[0], y=p[1], z=p[2]))
pub_msg.colors.append(ColorRGBA(r=1.0, a=1.0))
pub_msg.points.append(Point(x=q[0], y=q[1], z=q[2]))
pub_msg.colors.append(ColorRGBA(r=1.0, a=1.0))

pub = rospy.Publisher("testest", Marker, queue_size=1)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(pub_msg)
    r.sleep()

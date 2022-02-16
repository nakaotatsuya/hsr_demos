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
parser.add_argument("-l", "--label", type=str, default="microwave")
parser.add_argument("-n", "--number", type=list, default=[1,4])
args = parser.parse_args()

save_dir = osp.join(rospack.get_path(
    'action_pkg'), "sound_pos_data", args.label)

file_num = len(listdir(save_dir))
number_size = len(args.number)

pub_msg = Marker()
pub_msg2 = Marker()
for i in range(number_size):
    load_file = np.load(osp.join(save_dir, "{:0=5d}.npy").format(args.number[i]))
    print(load_file)

    p = load_file[0]
    v = load_file[1]
    q = p + v * 2


    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.header.frame_id = "map"
    pub_msg.type = 5

    pub_msg2.header.stamp = rospy.Time.now()
    pub_msg2.header.frame_id = "map"
    pub_msg2.type = 8

    pub_msg.scale.x = 0.01
    pub_msg.scale.y = 0.01
    pub_msg.scale.z = 0.01


    pub_msg.points.append(Point(x=p[0], y=p[1], z=p[2]))
    pub_msg.colors.append(ColorRGBA(r=1.0, a=1.0))
    pub_msg.points.append(Point(x=q[0], y=q[1], z=q[2]))
    pub_msg.colors.append(ColorRGBA(r=1.0, a=1.0))
    
p_list = np.empty((0,3))
v_list = np.empty((0,3))
for i in range(number_size):
    load_file = np.load(osp.join(save_dir, "{:0=5d}.npy").format(args.number[i]))
    p = load_file[0]
    v = load_file[1]
    p = p.reshape(-1,3)
    v = v.reshape(-1,3)
    p_list = np.append(p_list, p, axis=0)
    v_list = np.append(v_list, v, axis=0)

if number_size >= 2:
    pp = list(itertools.combinations(p_list, 2))
    vv = list(itertools.combinations(v_list, 2))
    print(pp[0])

    center_list = np.empty((0,3))
    for r, x in zip(pp, vv):
        p = r[0]
        v = x[0]
        
        q = r[1]
        w = x[1]

        s = sympy.Symbol("s")
        t = sympy.Symbol("t")

        pq2 = ((q[0] + t*w[0]) - (p[0] + s*v[0]))**2 + ((q[1] + t*w[1]) - (p[1] + s*v[1]))**2 + ((q[2] + t*w[2]) - (p[2] + s*v[2]))**2

        dpq2_ds = sympy.diff(pq2, s)
        dpq2_dt = sympy.diff(pq2, t)
        #print("dpq2_ds = {}".format(dpq2_ds))
        #print("dpq2_dt = {}".format(dpq2_dt))

        ans = sympy.solve([dpq2_ds, dpq2_dt])
        s, t = ans[s], ans[t]

        x1 ,y1, z1 = p[0] + s*v[0], p[1] + s*v[1], p[2] + s*v[2]
        #print(x1,y1,z1)
        x2 ,y2, z2 = q[0] + t*w[0], q[1] + t*w[1], q[2] + t*w[2]
        #print(x2,y2,z2)

        #pub_msg.points.append(Point(x=x1, y=y1, z=z1))
        #pub_msg.colors.append(ColorRGBA(b=1.0, a=1.0))
        #pub_msg.points.append(Point(x=x2, y=y2, z=z2))
        #pub_msg.colors.append(ColorRGBA(b=1.0, a=1.0))
    
        center = np.array([(x1+x2)/2 , (y1+y2)/2, (z1+z2)/2])

        pub_msg2.scale.x = 0.03
        pub_msg2.scale.y = 0.03
        pub_msg2.scale.z = 0.03
        pub_msg2.points.append(Point(x=center[0], y=center[1], z=center[2]))
        pub_msg2.colors.append(ColorRGBA(b=1.0, a=1.0))
        #print(center)
        center = center.reshape(-1,3)
        center_list = np.append(center_list, center, axis=0)
        
    print(center_list)
    
pub = rospy.Publisher("testest", Marker, queue_size=1)
pub2 = rospy.Publisher("testest2", Marker, queue_size=1)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(pub_msg)
    pub2.publish(pub_msg2)
    r.sleep()

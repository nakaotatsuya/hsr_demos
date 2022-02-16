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

#from tmc_eus_py.coordinates import Coordinates
from visualization_msgs.msg import Marker
from hark_msgs.msg import HarkSource
from sound_classification.msg import InSound
from geometry_msgs.msg import Point, PointStamped, Point, Quaternion
from std_msgs.msg import ColorRGBA, Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
sns.set(context="paper" , style ="whitegrid",rc={"figure.facecolor":"white"})

from scipy import linalg
import itertools
from sklearn import mixture
from sklearn.decomposition import PCA
from sklearn import datasets

import argparse

class VisualizedSoundPos():
    def __init__(self):
        rospack = rospkg.RosPack()

        parser = argparse.ArgumentParser()
        #parser.add_argument("-l", "--label", type=str, default="microwave")
        parser.add_argument("-l", "--labels", type=list, default=["microwave", "fridge", "door", "kettle", "dish", "bottle", "key"])
        #parser.add_argument("-l", "--labels", type=list, default=["bottle"])
        parser.add_argument("-g", "--gauss", type=bool, default=False)
        args = parser.parse_args()

        labels = args.labels

        colors = []
        for l in labels:
            c = np.random.rand(3)
            colors.append(c)

        colors = np.array([[0.0, 0.0, 0.0],
                           [0.0, 0.0, 1.0],
                           [0.0, 1.0, 1.0],
                           [1.0, 1.0, 1.0],
                           [1.0, 0.0, 0.0],
                           [1.0, 1.0, 0.0],
                           [0.0, 1.0, 0.0]])
        #pub_msg
        pub_msg = Marker()
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.header.frame_id = "map"
        pub_msg.type = 8
        
        for l, c in zip(labels, colors):
            self.label = l
            self.save_dir = osp.join(rospack.get_path(
                'action_pkg'), "sound_pos_data", self.label)

            center = np.load(osp.join(self.save_dir, "center.npy"), allow_pickle=True)
            pub_msg.scale.x = 0.1
            pub_msg.scale.y = 0.1
            pub_msg.scale.z = 0.1

            for i in range(center.shape[0]):
                if center[i][2] > 0.0:
                    pub_msg.points.append(Point(x = center[i][0], y=center[i][1], z=center[i][2]))
                    pub_msg.colors.append(ColorRGBA(r = c[0], g=c[1], b=c[2], a=1.0))
        pub = rospy.Publisher("~output", Marker, queue_size=1)

        #pub_gauss_msg
        if args.gauss:
            print("aaaaaaaaa")
            #labels = ["key", "microwave", "fridge", "kettle", "dish", "door", "bottle"]
            labels = ["fridge"]
            self.HEADER = Header()
            self.HEADER.stamp = rospy.Time.now()
            self.HEADER.frame_id = "map"
            
            POINTS = []
            x = np.arange(-1, 9, 0.05)
            y = np.arange(-20,-10, 0.05)
            X, Y = np.meshgrid(x,y)
            #print(X)
            z = np.c_[X.ravel(), Y.ravel()]
            #print(z.shape)

            pub_gauss_max_msgs = []
            pub_gauss_maxs = []
            for j, label in enumerate(labels):
                load_file = np.load(osp.join(rospack.get_path("action_pkg"), "sound_pos_data", label, "center.npy"), allow_pickle=True)
                print(load_file.shape)
                cvtype, component = self.calculate_BIC(load_file)
                means, covars, weights, means_3d, covars_3d = self.best_BIC_GMM(load_file, cvtype, component)
                np.save(osp.join(rospack.get_path("action_pkg"), "gauss_data", label, "weights.npy"), np.array(weights))
                np.save(osp.join(rospack.get_path("action_pkg"), "gauss_data", label, "means_3d.npy"), np.array(means_3d))
                np.save(osp.join(rospack.get_path("action_pkg"), "gauss_data", label, "covars_3d.npy"), np.array(covars_3d))
                #print(means)
                #print(covars)
                #print(weights)
                for i in range(len(weights)):
                    if i == 0:
                        Z = weights[0] * self.gaussian(z, means[0], covars[0])
                    else:
                        Z += weights[i] * self.gaussian(z, means[i], covars[i])
                Z = Z.reshape(X.shape)
                X = X.flatten()
                Y = Y.flatten()
                Z = Z.flatten()
                #print(Z.shape)
                g = np.vstack((X, Y, Z)).T
                cos = [0xff0000, 0x00ff00, 0xffff00, 0xff00ff, 0xf0ffff, 0x0000ff, 0x00ffff]
                co = np.array([cos[j]] * 40000)
                #print(co.shape)
                #print(g.shape)
                g = np.vstack((g.T, co.T)).T
                #print(g.shape)
                g_sort = g[np.argsort(g[:, 2])[::-1]]
                print(g_sort[0])
                pub_gauss_max_msg = PointStamped()
                pub_gauss_max_msg.header = self.HEADER
                pub_gauss_max_msg.point = Point(x=g_sort[0][0], y=g_sort[0][1], z=g_sort[0][2])
                pub_gauss_max_msgs.append(pub_gauss_max_msg)
                pub_gauss_max = rospy.Publisher("~output_gauss_max_{}".format(label), PointStamped, queue_size=1)
                pub_gauss_maxs.append(pub_gauss_max)
                POINTS.extend(g.tolist())
                #print(len(POINTS))
                self.FIELDS =[
                    PointField(name="x", offset=0, datatype=7, count=1),
                    PointField(name="y", offset=4, datatype=7, count=1),
                    PointField(name="z", offset=8, datatype=7, count=1),
                    PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
                #PointField(name="pfh", offset=12, datatype=PointField.FLOAT32, count=1),
                #PointField(name="w", offset=12, datatype=7, count=1),
                ]
            #print("point", len(POINTS))
            pub_gauss_msg = pc2.create_cloud(self.HEADER, self.FIELDS, POINTS)
            pub_gauss = rospy.Publisher("~output_gauss", PointCloud2, queue_size=1)


            a = np.arange(-1, 9, 1)
            b = np.arange(-20, -10, 1)
            c = np.arange(0, 2, 0.2)
            A, B, C = np.meshgrid(a, b, c)
            d = np.c_[A.ravel(), B.ravel(), C.ravel()]
            for i in range(len(weights)):
                if i == 0:
                    D = weights[0] * self.gaussian(d , means_3d[0], covars_3d[0])
                else:
                    D += weights[i] * self.gaussian(d , means_3d[i], covars_3d[i])
            D = D.reshape(A.shape)
            A = A.flatten()
            B = B.flatten()
            C = C.flatten()
            D = D.flatten()
            #print(D.shape)
            #print(D.max())
            
            # h = np.vstack((A,B,C,D)).T
            # print(h.shape)
            # POINTS.extend(h.tolist())
            # pub_gauss_msg = pc2.create_cloud(self.HEADER, self.FIELDS, POINTS)
            # pub_gauss = rospy.Publisher("~output_gauss", PointCloud2, queue_size=1)
            
            
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            pub.publish(pub_msg)
            if args.gauss:
                pub_gauss.publish(pub_gauss_msg)
                for i, label in enumerate(labels):
                    pub_gauss_maxs[i].publish(pub_gauss_max_msgs[i])
            r.sleep()

    def gaussian(self, x, mu, sigma):
        #分散共分散行列の行列式
        det = np.linalg.det(sigma)
        #print(det)
        #分散共分散行列の逆行列
        inv = np.linalg.inv(sigma)
        n = x.ndim
        #print(inv.shape)

        #print((x - mu).shape)
        return np.exp(-np.diag(np.dot(np.dot((x - mu),inv),(x - mu).T))/2.0) / (np.sqrt((2 * np.pi) ** n * det))

    def best_BIC_GMM(self, X, cvtype, component):
        gmm = mixture.GMM(n_components=component, covariance_type=cvtype)
        gmm.fit(X)
        #print(gmm.means_.shape)
        #print(gmm.covars_.shape)
        #print(gmm.weights_.shape)

        #z成分無視(2次元に投影するため)
        means = gmm.means_[:, 0:2]
        covars = gmm.covars_[:, 0:2, 0:2]
        weights = gmm.weights_

        a =gmm.means_
        b =gmm.covars_
        return means, covars, weights, a, b

    def calculate_BIC(self, X):
        lowestBIC = np.infty
        bic = []
        nComponentsRange = range(1,6)
        #cvTypes = ["spherical", "tied", "diag", "full"]
        cvTypes = ["full"]
        nInit = 10
        for cvType in cvTypes:
            for nComponents in nComponentsRange:
                gmm = mixture.GMM(n_components=nComponents,
                                  covariance_type=cvType, n_init=nInit)
                gmm.fit(X)
                bic.append(gmm.bic(X))
                if bic[-1] < lowestBIC:
                    lowestBIC = bic[-1]
                    bestGmm = gmm
        bic = np.array(bic)
        #print(bic)
        colorIter = itertools.cycle(["navy", "turquoise", "cornflowerblue", "darkorange"])

        bars = []
        plt.figure(figsize=(8, 6),dpi=100)
        ax = plt.subplot(111)
        for i, (cvType, color) in enumerate(zip(cvTypes, colorIter)):
            xpos = np.array(nComponentsRange) + .2 * (i - 2)
            bars.append(plt.bar(xpos, bic[i * len(nComponentsRange):
                                          (i + 1) * len(nComponentsRange)],
                                width=.2, color=color))
        plt.xticks(nComponentsRange)
        plt.ylim([bic.min() * 1.01 - .01 * bic.max(), bic.max()])
        plt.title('BIC score per model')
        xpos = np.mod(bic.argmin(), len(nComponentsRange)) + .65 + .2 * np.floor(bic.argmin() / len(nComponentsRange))
        plt.text(xpos, bic.min() * 0.97 + .03 * bic.max(), '*', fontsize=14)
        ax.set_xlabel('Number of components')
        ax.legend([b[0] for b in bars], cvTypes)

        #plt.show()
        #bic.reshape(len(cvTypes), len(nComponentsRange))
        best_bic_idx = np.argmin(bic)
        #print(best_bic_idx)
        best_cvtypes_idx = best_bic_idx / len(nComponentsRange)
        best_ncomponents_idx = best_bic_idx % len(nComponentsRange)

        best_cvtypes = cvTypes[best_cvtypes_idx]
        best_ncomponents = nComponentsRange[best_ncomponents_idx]
        print(best_cvtypes, best_ncomponents)
        return best_cvtypes, best_ncomponents

if __name__ == "__main__":
    rospy.init_node("visualized_sound_pos")
    visualized_sound_pos = VisualizedSoundPos()

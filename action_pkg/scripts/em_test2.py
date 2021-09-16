#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

import seaborn as sns
sns.set(context="paper" , style ="whitegrid",rc={"figure.facecolor":"white"})

from scipy import linalg
import itertools
from sklearn import mixture
from sklearn.decomposition import PCA
from sklearn import datasets

import matplotlib.mlab as mlab
from mpl_toolkits.mplot3d import axes3d, Axes3D
from matplotlib import cm

dataset = datasets.load_iris()
dic = {}
for k,v in zip(dataset.feature_names, dataset.data.T):
    dic[k] = v
df = pd.DataFrame(dic)

#print(df.head().T)

X = df.values
#print(X)
lowestBIC = np.infty
bic = []
nComponentsRange = range(1,7)
cvTypes = ["spherical", "tied", "diag", "full"]
nInit = 10
print(X.shape)
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
colorIter = itertools.cycle(["navy", "turquoise", "cornflowerblue", "darkorange"])
print(bic.shape)
print(bic)

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

plt.show()

#----------------------------------


# gmm = mixture.GMM(n_components=3, covariance_type="full")
# gmm.fit(X)
# print(gmm.weights_.shape)
# print(gmm.means_.shape)
# means = gmm.means_[0][0:2]
# print(means.shape)
# print(gmm.covars_)
# covars = gmm.covars_[0, 0:2, 0:2]
# print(covars)

# x = y = np.arange(-20,20, 0.5)
# X, Y = np.meshgrid(x,y)

# z = np.c_[X.ravel(), Y.ravel()]
# #print(z.shape)

# def gaussian(x, mu, sigma):
#     #分散共分散行列の行列式
#     det = np.linalg.det(sigma)
#     print(det)
#     #分散共分散行列の逆行列
#     inv = np.linalg.inv(sigma)
#     n = x.ndim
#     print(inv.shape)

#     print((x - mu).shape)
#     return np.exp(-np.diag(np.dot(np.dot((x - mu),inv),(x - mu).T))/2.0) / (np.sqrt((2 * np.pi) ** n * det))

# Z = gaussian(z, means, covars)
# Z = Z.reshape(X.shape)
# print(Z.shape)

# fig = plt.figure(figsize=(15,15))
# ax = fig.add_subplot(111, projection="3d")

# ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm)
# plt.show()

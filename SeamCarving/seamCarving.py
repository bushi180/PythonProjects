# -*- coding: utf-8 -*-
"""
Created on Tue Sep 26 00:46:52 2017

@author: David
"""

import cv2
import glob
import imageio
import seamCarvingHelper as scHelper

# Seam removal.
inputImage = cv2.imread("inputImages/fig4.png")
result = scHelper.carveNum(0,50,inputImage,1)

# Create gif.
plotList = []
plots = glob.glob('gifFolder/remove/*.png')
for plot in plots:
    plotList.append(imageio.imread(plot))
imageio.mimsave('seamRemoving.gif', plotList, fps=10)

# Seam insertion.
inputImage = cv2.imread("inputImages/fig8.png")
# First step 50% insertion.
result = scHelper.vAdd(inputImage,119)

# Create gif.
plotList = []
plots = glob.glob('gifFolder/add/*.png')
for plot in plots:
    plotList.append(imageio.imread(plot))
imageio.mimsave('seamAdding.gif', plotList, fps=10)
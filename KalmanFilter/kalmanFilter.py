# -*- coding: utf-8 -*-
"""
Created on Sat Apr 25 21:57:25 2020

@author: David
"""

import numpy as np
import kalmanFilterHelper as kfHelper
import glob
import imageio
import os

# Setup
inputDir = "inputImages"
outputDir = "output"
noise = {'x': 1.5, 'y': 1.5}
initPos = {'x': 93, 'y': 245}

# Process noise and measurement noise arrays.
Q = 0.5 * np.eye(4)
R = 0.1 * np.eye(2)

# Run Kalman filter.
kf = kfHelper.KalmanFilter(initPos['x'], initPos['y'], Q, R)
kfHelper.filterRun(kf, os.path.join(inputDir, "crosswalk"), noise)

# Create gif.
plotList = []
plots = glob.glob('output/*.png')
for plot in plots:
    plotList.append(imageio.imread(plot))
imageio.mimsave('kalmanFilter.gif', plotList, fps=29.97)

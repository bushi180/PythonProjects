# -*- coding: utf-8 -*-
"""
Created on Tue Dec  4 15:21:52 2018

@author: David
"""
import cv2
import stereoCorrespondenceHelper as scHelper

#Tsukuba wSSD
print ""
print "Starting Tsukuba map"
print ""

temp = cv2.imread("inputImages/studio/left.png")
img = cv2.imread("inputImages/studio/right.png")

# Create disparity maps.
disparityLeft, disparityRight, disparityOcculsion = scHelper.disparityMap(temp, img)

# Save.
cv2.imwrite("output/tsukuba_leftD.png", disparityLeft)
cv2.imwrite("output/tsukuba_rightD.png", disparityRight)
cv2.imwrite("output/tsukuba_occludedD.png", disparityOcculsion)

print ""
print "Starting Motorcycle map"
print ""

temp = cv2.imread("inputImages/motorcycle/left.png")
img = cv2.imread("inputImages/motorcycle/right.png")

# Create disparity maps.
disparityLeft, disparityRight, disparityOcculsion = scHelper.disparityMap(temp, img)

# Save.
cv2.imwrite("output/motorcycle_leftD.png", disparityLeft)
cv2.imwrite("output/motorcycle_rightD.png", disparityRight)
cv2.imwrite("output/motor_occludedD.png", disparityOcculsion)
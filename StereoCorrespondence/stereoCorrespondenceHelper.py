# -*- coding: utf-8 -*-
"""
Created on Wed Apr 29 01:41:52 2020

@author: David
"""

import cv2
import numpy as np

def windowSumSqDiff(leftImg,rightImg):
    # Convert to grayscale.
    leftGray = cv2.cvtColor(leftImg, cv2.COLOR_BGR2GRAY) / 255.0
    rightGray = cv2.cvtColor(rightImg, cv2.COLOR_BGR2GRAY) / 255.0
    
    windowSize = 15
    
    # Add padding.
    pad = windowSize // 2
    tempB = cv2.copyMakeBorder(leftGray,pad,pad,pad,pad,cv2.BORDER_CONSTANT,value=[0,0,0])
    imgB = cv2.copyMakeBorder(rightGray,pad,pad,pad,pad,cv2.BORDER_CONSTANT,value=[0,0,0])
    
    padPlus = pad + 1
    maxDisparity = 100
    halfMax = maxDisparity // 2
    disparity = np.zeros(leftGray.shape)
    
    # Create left windows.
    leftWinList = []
    for lRow in range(pad, tempB.shape[0] - pad):
        for lCol in range(pad, tempB.shape[1] - pad):
            
            leftPatch = tempB[lRow - pad : lRow + padPlus, 
                              lCol - pad : lCol + padPlus]
            leftWinList.append((leftPatch,lRow-pad))
    
    # Create right windows.
    rightWinRowList = []
    for lRow in range(pad, imgB.shape[0] - pad):
        
        rightStrip = imgB[lRow - pad : lRow + padPlus, : ]
        rowList = []
        
        for rCol in range(pad, rightStrip.shape[1] - pad):

            rightPatch = rightStrip[ : , rCol - pad : rCol + padPlus]
            rowList.append(rightPatch)
        rightWinRowList.append(rowList)

    # Create disparity map.
    lCol = 0
    disparity = []
    for i in range(len(leftWinList)):
        leftP = leftWinList[i][0]
        row = leftWinList[i][1]
        rightStrip = rightWinRowList[row]
        
        if lCol < halfMax:
            rang = (0, maxDisparity)
            
        elif lCol > (leftGray.shape[1]-1) - halfMax:
            rang = (leftGray.shape[1] - maxDisparity, leftGray.shape[1])
            
        else:
            rang = (lCol - halfMax, lCol + halfMax)
        
        colList = []
        for rCol in range(rang[0], rang[1]): 
            rightPatch = rightStrip[rCol]
            colList.append(rightPatch)

        leftPRepeat = np.asarray([leftP]*len(colList))
        rightPWin = np.array(colList)
        ssdList = np.sum(np.sum((leftPRepeat - rightPWin)**2, 1),1)
        
        if lCol < halfMax:
            bestRCol = maxDisparity - (maxDisparity - np.argmin(ssdList))
            
        elif lCol > (leftGray.shape[1]-1) - halfMax:
            bestRCol = (leftGray.shape[1] - maxDisparity) + np.argmin(ssdList)
            
        else:
            bestRCol = (np.argmin(ssdList) - halfMax) + lCol

        d = np.absolute(lCol - bestRCol)
        disparity.append(d)
        
        if lCol < leftGray.shape[1] - 1:
            lCol += 1
        else:
            lCol = 0

    disparity = np.array(disparity).reshape(leftGray.shape).astype(np.float32)
    
    return disparity

#------------------------------------------------------------------------------
    
def disparityMap(temp, img):
    print "Creating Left"
    disparityLeft = windowSumSqDiff(temp, img)
    print "Left done"
    print ""
    
    print "Creating Right"
    disparityRight = windowSumSqDiff(img, temp)
    print "Right done"
    print ""
    
    # Find occulsions.
    disparityOcculsion = np.zeros(disparityLeft.shape)        
    for row in range(disparityLeft.shape[0]):
        for col in range(disparityLeft.shape[1]):
            if abs(disparityLeft[row,col] - disparityRight[row,col]) <= 1:
                disparityOcculsion[row,col] = disparityLeft[row,col]
    
    # Normalize and blur.
    disparityLeft = (cv2.normalize(disparityLeft,None,0,255,cv2.NORM_MINMAX).astype(np.uint8) * 5) + 20
    disparityRight = (cv2.normalize(disparityRight,None,0,255,cv2.NORM_MINMAX).astype(np.uint8) * 5) + 20
    disparityOcculsion = (cv2.normalize(disparityOcculsion,None,0,255,cv2.NORM_MINMAX).astype(np.uint8) * 2) + 10    
    disparityLeftBlur = cv2.medianBlur(disparityLeft, 5)
    disparityRightBlur = cv2.medianBlur(disparityRight, 5)
    disparityOcculsionBlur = cv2.medianBlur(disparityOcculsion, 5)
    
    return disparityLeftBlur, disparityRightBlur, disparityOcculsionBlur
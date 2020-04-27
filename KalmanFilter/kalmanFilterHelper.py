# -*- coding: utf-8 -*-
"""
Created on Sat Apr 25 21:55:00 2020

@author: David
"""
import numpy as np
import os
import cv2
import imutils

class KalmanFilter(object):

    def __init__(self, init_x, init_y, Q=0.1 * np.eye(4), R=0.1 * np.eye(2)):
        # X
        self.state = np.array([init_x, init_y, 0., 0.])
        
        # E
        self.covariance = np.array([[0.,0.,0.,0.],
                                    [0.,0.,0.,0.],
                                    [0.,0.,0.,0.],
                                    [0.,0.,0.,0.]])
        # Dt
        self.stateTransMatrix = np.array([[1.,0.,0.,0.],
                                          [0.,1.,0.,0.],
                                          [0.,0.,0.,0.],
                                          [0.,0.,0.,0.]])
        # Mt
        self.measMatrix = np.array([[1.,0.,0.,0.],
                                    [0.,1.,0.,0.]])
        # Edt
        self.procNoiseMatrix = Q
        
        # Emt
        self.measNoiseMatrix = R

    def predict(self):
        
        sTMat = self.stateTransMatrix
        self.state = np.dot(sTMat, self.state)
        self.covariance = np.dot(np.dot(sTMat, self.covariance), sTMat.T) + self.procNoiseMatrix

    def correct(self, meas_x, meas_y):
        
        mMat = self.measMatrix
        measurements = np.array([meas_x, meas_y])
        identityM = np.eye(4)
        kalGain = np.dot( np.dot(self.covariance, mMat.T), 
                         np.linalg.pinv( np.dot(np.dot(mMat, self.covariance), mMat.T) 
                         + self.measNoiseMatrix ) )
        
        self.state = self.state + np.dot(kalGain, (measurements - np.dot(mMat, self.state)))
        self.covariance = np.dot((identityM - np.dot(kalGain, mMat)), self.covariance)

    def process(self, measurement_x, measurement_y):

        self.predict()
        self.correct(measurement_x, measurement_y)

        return self.state[0], self.state[1]

#------------------------------------------------------------------------------
    
def filterRun(kf, imgsDir, noise):

    imgsList = [f for f in os.listdir(imgsDir)
                 if f[0] != '.' and f.endswith('.jpg')]
    imgsList.sort()

    # Measurement.
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    frameNum = 0
    for img in imgsList:

        frame = cv2.imread(os.path.join(imgsDir, img))
        frame = imutils.resize(frame, width=min(500, frame.shape[1]))

        # HOG Sensor.
        rects, weights = hog.detectMultiScale(frame, winStride=(4, 4),
                                                padding=(4, 4), scale=1.05)
        if len(weights) > 0:
            maxWeightId = np.argmax(weights)
            h_x, h_y, h_w, h_h = rects[maxWeightId]

            h_x += h_w // 2
            h_y += h_h // 2

            h_x += np.random.normal(0, noise['x'])
            h_y += np.random.normal(0, noise['y'])

        x, y = kf.process(h_x, h_y)

        # Draw bounding box and output frame.
        outFrame = frame.copy()
        cv2.rectangle(outFrame, (int(x) - h_w // 2, int(y) - h_h // 2),
                      (int(x) + h_w // 2, int(y) + h_h // 2),
                      (0, 255, 0), 2)
        cv2.imwrite('output/crosswalk_%03d.png' % frameNum, outFrame)
        frameNum += 1
        if frameNum % 20 == 0:
            print 'Still working - at frame %d' % frameNum
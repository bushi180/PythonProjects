# -*- coding: utf-8 -*-
"""
Created on Mon Mar  2 21:37:54 2020

@author: David
"""

import numpy as np
from math import cos
from math import sin
from math import pi
from math import atan2
import random
import hashlib

noiseFlag = True
moveNoise = 0.01
hashSeed = 'some_seed'

def correctAngle(t):
    correction = ((t + pi) % (2 * pi)) - pi
    return correction

def calculateBearing(p, q):
    x1, y1 = p
    x2, y2 = q
    dx = x2 - x1
    dy = y2 - y1
    return atan2(dy, dx)

class Slam:
    def __init__(self):
        self.initial_pos = 0.0
        self.size = 2
        self.omega = np.zeros((self.size, self.size))
        self.xI = np.zeros((self.size, 1))
        self.foundLandmarks = {}
        self.landmarkIndex = 0
        self.heading = 0.0
        
        # Initialize
        self.omega[0, 0] = self.omega[1, 1] = 1.0
        self.xI[0, 0] = self.xI[1, 0] = self.initial_pos

    def processMeasurements(self, measurements): 
        # Update for new landmarks
        expandNum = 0
        for key in measurements:
            if key not in self.foundLandmarks:
                self.foundLandmarks[key] = self.landmarkIndex
                self.landmarkIndex += 1
                expandNum += 1
        
        # Expand omega and xI for new landmarks
        if expandNum > 0:
            eNum = expandNum * 2
            col = np.zeros([self.size, eNum])
            self.omega = np.append(self.omega, col, axis=1)
            self.size += eNum
            row = np.zeros([eNum, self.size])
            self.omega = np.append(self.omega, row, axis=0)
            row2 = np.zeros([eNum, 1])
            self.xI = np.append(self.xI, row2, axis=0)

        # Update values
        measurementNoise = 1.0
        measureUpPos, measureUpNeg = 1.0 / measurementNoise, -1.0 / measurementNoise
    
        # Measurement Update
        rIndex = 0
        for key in measurements:
            lIndex = (1 + self.foundLandmarks[key]) * 2

            # Update omega
            self.omega[lIndex, lIndex] += measureUpPos
            self.omega[lIndex+1, lIndex+1] += measureUpPos
            self.omega[rIndex, rIndex] += measureUpPos
            self.omega[rIndex+1, rIndex+1] += measureUpPos
            self.omega[lIndex, rIndex] += measureUpNeg
            self.omega[lIndex+1, rIndex+1] += measureUpNeg
            self.omega[rIndex, lIndex] += measureUpNeg
            self.omega[rIndex+1, lIndex+1] += measureUpNeg
            
            # Find landmark coordinates
            landDistance = measurements[key]['distance']
            landBearing = correctAngle(self.heading + measurements[key]['bearing'])
            dx = landDistance * cos(landBearing)
            dy = landDistance * sin(landBearing)

            # Update xI
            self.xI[rIndex, 0] += -dx / measurementNoise
            self.xI[rIndex+1, 0] += -dy / measurementNoise
            self.xI[lIndex, 0] += dx / measurementNoise
            self.xI[lIndex+1, 0] += dy / measurementNoise
        
        # Find estimate bot x, y before movement
        mu = np.dot(np.linalg.inv(self.omega), self.xI)
        x, y = mu[0][0], mu[1][0]

        return (x, y), mu
    
    def processMovement(self, steering, distance, motionNoise):
        # Update values
        motionUpPos, motionUpNeg = 1.0 / motionNoise, -1.0 / motionNoise
        
        # Expand
        self.omega = np.insert(self.omega, 2, 0, axis=1)
        self.omega = np.insert(self.omega, 2, 0, axis=1)
        self.omega = np.insert(self.omega, 2, 0, axis=0)
        self.omega = np.insert(self.omega, 2, 0, axis=0)
        self.xI = np.insert(self.xI, 2, 0, axis=0)
        self.xI = np.insert(self.xI, 2, 0, axis=0)

        rIndex = 0
        
        # Motion Update
        self.omega[rIndex, rIndex] += motionUpPos
        self.omega[rIndex+1, rIndex+1] += motionUpPos
        self.omega[rIndex+2, rIndex+2] += motionUpPos
        self.omega[rIndex+3, rIndex+3] += motionUpPos
        self.omega[rIndex+2, rIndex] += motionUpNeg
        self.omega[rIndex+3, rIndex+1] += motionUpNeg
        self.omega[rIndex, rIndex+2] += motionUpNeg
        self.omega[rIndex+1, rIndex+3] += motionUpNeg
        
        # Find bot coordinates after move
        self.heading = correctAngle(self.heading + float(steering))
        dx = distance * cos(self.heading)
        dy = distance * sin(self.heading)

        # Update xi
        self.xI[rIndex, 0] += -dx / motionNoise
        self.xI[rIndex+1, 0] += -dy / motionNoise
        self.xI[rIndex+2, 0] += dx / motionNoise
        self.xI[rIndex+3, 0] += dy / motionNoise
        
        # Take
        b = np.delete(self.omega, range(2,len(self.omega)), 1)
        b = np.delete(b, range(2,len(b)), 0)
        a = np.delete(self.omega, [0,1], 1)
        a = np.delete(a, range(2,len(a)), 0)
        c = np.delete(self.xI, range(2,len(self.xI)), 0)

        omegaP = np.delete(self.omega, [0,1], 1)
        omegaP = np.delete(omegaP, [0,1], 0)
        xiP = np.delete(self.xI, [0,1], 0)
        
        # Calculate
        self.omega = omegaP - (np.dot(np.dot(np.transpose(a), np.linalg.inv(b)), a))
        self.xI = xiP - (np.dot(np.dot(np.transpose(a), np.linalg.inv(b)), c))

        # Find new bot x, y after movement
        mu = np.dot(np.linalg.inv(self.omega), self.xI)
        x, y = mu[0][0], mu[1][0]

        return x, y

#------------------------------------------------------------------------------
        
class State:
    def __init__(self, areaMap, maxDistance=1.0, maxSteering=pi/2.+0.01, horizonDistance=3):
        self.reachedLocations = list()
        self.maxDistance = maxDistance
        self.maxSteering = maxSteering
        self.horizonDistance = horizonDistance
        self.found = list()
        self.rows = len(areaMap)
        self.cols = len(areaMap[0])
        self.landmarks = list()
        self.startPosition = dict()
        
        # Now process the interior of the provided map
        for i in range(self.rows):
            for j in range(self.cols):
                thisSquare = areaMap[i][j]
                x, y = float(j), -float(i)

                # Process landmarks
                if thisSquare == 'L':
                    landmark = dict()
                    landmark['x'] = x
                    landmark['y'] = y

                    self.landmarks.append(landmark)

                # Process start
                if thisSquare == '@':
                    self.startPosition['x'] = x + 0.5
                    self.startPosition['y'] = y - 0.5

        # initialize the bot at the start position and at a bearing pointing due east
        self.bot = Bot(x=self.startPosition['x'], y=self.startPosition['y'], bearing=0.0,
                                 maxDistance=self.maxDistance, maxSteering=self.maxSteering)

    def createMeasurements(self, noise=noiseFlag):
        measurements = dict()

        # process landmarks
        for location in self.landmarks:
            distance, bearing = self.bot.measureDistanceBearingTo((location['x'], location['y']), noise=noise)

            if distance <= self.horizonDistance:
                if (location['x'], location['y']) not in self.found:
                    self.found.append((location['x'], location['y']))
                measurements[int(hashlib.md5(str(location) + hashSeed).hexdigest(), 16)] = {'distance': distance,
                                                                                             'bearing': bearing,
                                                                                             'type': 'beacon'}
        return measurements,self.found

    def actionUpdate(self, action, noise=noiseFlag):
        action = action.split()
        actionType = action[0]

        if actionType == 'move':
            steering, distance = action[1:]
            self._attemptMove(float(steering), float(distance), noise=noise)
        else:
            raise Exception('improperly formatted action: {}'.format(''.join(action)))
            
    def _attemptMove(self, steering, distance, noise=noiseFlag):
        try:
            distanceGood = 0.0 <= distance <= self.maxDistance
            steeringGood = (-self.maxSteering) <= steering <= self.maxSteering

            if noise:
                steering += random.uniform(-moveNoise, moveNoise)
                distance *= random.uniform(1.0 - moveNoise, 1.0 + moveNoise)

            if distanceGood and steeringGood:
                self.bot.move(steering, distance)

        except ValueError:
            raise Exception('improperly formatted move: {} {}'.format(steering, distance))

#------------------------------------------------------------------------------
            
class Bot:
    def __init__(self, x=0.0, y=0.0, bearing=0.0, maxDistance=1.0, maxSteering=pi/4):
        self.x = x
        self.y = y
        self.bearing = bearing
        self.maxDistance = maxDistance
        self.maxSteering = maxSteering

    def set_noise(self, steeringNoise, distanceNoise, measurementNoise):
        self.steeringNoise = float(steeringNoise)
        self.distanceNoise = float(distanceNoise)
        self.measurementNoise = float(measurementNoise)

    # move the bot
    def move(self, steering, distance, noise=False):
        if noise:
            steering += random.uniform(-0.01, 0.01)
            distance *= random.uniform(0.99, 1.01)

        steering = max(-self.maxSteering, steering)
        steering = min(self.maxSteering, steering)
        distance = max(0, distance)
        distance = min(self.maxDistance, distance)

        self.bearing = correctAngle(self.bearing + float(steering))
        self.x += distance * cos(self.bearing)
        self.y += distance * sin(self.bearing)

    def measureDistanceBearingTo(self, point, noise=False):
        currentPosition = (self.x, self.y)

        distanceTo = np.linalg.norm(np.array(currentPosition) - np.array(point))
        bearingTo = calculateBearing(currentPosition, point)

        if noise:
            distanceSigma = 0.05 * distanceTo
            bearingSigma = 0.02 * distanceTo
            distanceNoise = random.gauss(0, distanceSigma)
            bearingNoise = random.gauss(0, bearingSigma)
        else:
            distanceNoise = 0
            bearingNoise = 0

        measuredDistance = distanceTo + distanceNoise
        measuredBearing = correctAngle(bearingTo - self.bearing + bearingNoise)

        return (measuredDistance, measuredBearing)
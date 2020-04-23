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

NOISE_FLAG = True
NOISE_MOVE = 0.01
HASH_SEED = 'some_seed'

def truncate_angle(t):
    return ((t+pi) % (2*pi)) - pi

def compute_bearing(p, q):
    x1, y1 = p
    x2, y2 = q
    dx = x2 - x1
    dy = y2 - y1
    return atan2(dy, dx)

class State:
    def __init__(self, area_map, max_distance=1.0, max_steering=pi/2.+0.01, horizon_distance=3):
        self.reached_locations = list()
        self.max_distance = max_distance
        self.max_steering = max_steering
        self.horizon_distance = horizon_distance
        self.found = list()
        self.rows = len(area_map)
        self.cols = len(area_map[0])
        self.landmarks = list()
        self._start_position = dict()
        
        # Now process the interior of the provided map
        for i in range(self.rows):
            for j in range(self.cols):
                this_square = area_map[i][j]
                x, y = float(j), -float(i)

                # Process landmarks
                if this_square == 'L':
                    landmark = dict()
                    landmark['x'] = x
                    landmark['y'] = y

                    self.landmarks.append(landmark)

                # Process start
                if this_square == '@':
                    self._start_position['x'] = x + 0.5
                    self._start_position['y'] = y - 0.5

        # initialize the bot at the start position and at a bearing pointing due east
        self.bot = Bot(x=self._start_position['x'], y=self._start_position['y'], bearing=0.0,
                                 max_distance=self.max_distance, max_steering=self.max_steering)

    def generate_measurements(self, noise=NOISE_FLAG):
        measurements = dict()

        # process landmarks
        for location in self.landmarks:
            distance, bearing = self.bot.measure_distance_and_bearing_to((location['x'], location['y']), noise=noise)

            if distance <= self.horizon_distance:
                if (location['x'], location['y']) not in self.found:
                    self.found.append((location['x'], location['y']))
                measurements[int(hashlib.md5(str(location) + HASH_SEED).hexdigest(), 16)] = {'distance': distance,
                                                                                             'bearing': bearing,
                                                                                             'type': 'beacon'}
        return measurements,self.found

    def update_according_to(self, action, noise=NOISE_FLAG):
        action = action.split()
        action_type = action[0]

        if action_type == 'move':
            steering, distance = action[1:]
            self._attempt_move(float(steering), float(distance), noise=noise)
        else:
            raise Exception('improperly formatted action: {}'.format(''.join(action)))
            
    def _attempt_move(self, steering, distance, noise=NOISE_FLAG):
        try:
            distance_ok = 0.0 <= distance <= self.max_distance
            steering_ok = (-self.max_steering) <= steering <= self.max_steering

            if noise:
                steering += random.uniform(-NOISE_MOVE, NOISE_MOVE)
                distance *= random.uniform(1.0 - NOISE_MOVE, 1.0 + NOISE_MOVE)

            if distance_ok and steering_ok:
                self.bot.move(steering, distance)

        except ValueError:
            raise Exception('improperly formatted move command : {} {}'.format(steering, distance))

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

    def process_measurements(self, measurements): 
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
        measurement_noise = 1.0
        measureUpPos, measureUpNeg = 1.0 / measurement_noise, -1.0 / measurement_noise
    
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
            landBearing = truncate_angle(self.heading + measurements[key]['bearing'])
            dx = landDistance * cos(landBearing)
            dy = landDistance * sin(landBearing)

            # Update xI
            self.xI[rIndex, 0] += -dx / measurement_noise
            self.xI[rIndex+1, 0] += -dy / measurement_noise
            self.xI[lIndex, 0] += dx / measurement_noise
            self.xI[lIndex+1, 0] += dy / measurement_noise
        
        # Find estimate bot x, y before movement
        mu = np.dot(np.linalg.inv(self.omega), self.xI)
        x, y = mu[0][0], mu[1][0]

        return (x, y), mu
    
    def process_movement(self, steering, distance, motion_noise):
        # Update values
        motionUpPos, motionUpNeg = 1.0 / motion_noise, -1.0 / motion_noise
        
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
        self.heading = truncate_angle(self.heading + float(steering))
        dx = distance * cos(self.heading)
        dy = distance * sin(self.heading)

        # Update xi
        self.xI[rIndex, 0] += -dx / motion_noise
        self.xI[rIndex+1, 0] += -dy / motion_noise
        self.xI[rIndex+2, 0] += dx / motion_noise
        self.xI[rIndex+3, 0] += dy / motion_noise
        
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
    
class Bot:
    def __init__(self, x=0.0, y=0.0, bearing=0.0, max_distance=1.0, max_steering=pi/4):
        self.x = x
        self.y = y
        self.bearing = bearing
        self.max_distance = max_distance
        self.max_steering = max_steering

    def set_noise(self, steering_noise, distance_noise, measurement_noise):
        self.steering_noise = float(steering_noise)
        self.distance_noise = float(distance_noise)
        self.measurement_noise = float(measurement_noise)

    # move the bot
    def move(self, steering, distance, noise=False):
        if noise:
            steering += random.uniform(-0.01, 0.01)
            distance *= random.uniform(0.99, 1.01)

        steering = max(-self.max_steering, steering)
        steering = min(self.max_steering, steering)
        distance = max(0, distance)
        distance = min(self.max_distance, distance)

        self.bearing = truncate_angle(self.bearing + float(steering))
        self.x += distance * cos(self.bearing)
        self.y += distance * sin(self.bearing)

    def measure_distance_and_bearing_to(self, point, noise=False):

        current_position = (self.x, self.y)

        distanceTo = np.linalg.norm(np.array(current_position) - np.array(point))
        bearingTo = compute_bearing(current_position, point)

        if noise:
            distance_sigma = 0.05 * distanceTo
            bearing_sigma = 0.02 * distanceTo

            distance_noise = random.gauss(0, distance_sigma)
            bearing_noise = random.gauss(0, bearing_sigma)
        else:
            distance_noise = 0
            bearing_noise = 0

        measured_distance = distanceTo + distance_noise
        measured_bearing = truncate_angle(
            bearingTo - self.bearing + bearing_noise)

        return (measured_distance, measured_bearing)
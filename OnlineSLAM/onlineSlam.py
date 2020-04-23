# -*- coding: utf-8 -*-
"""
Created on Thu Jan  9 22:02:59 2020

@author: David
"""
import onlineSlamHelper as osHelp
from matplotlib import pyplot as plt
from math import degrees
import numpy as np
import glob
import imageio

# [Instruction, turn amount, distance]
moves = ['move -1.570963 0.5','move -1.570963 0.5','move 0.0 1.0','move 0.0 1.0',
         'move 1.570963 1.0','move 0.0 1.0','move 0.0 1.0','move 0.0 1.0',
         'move 1.570963 1.0','move -1.570963 1.0','move 0.0 1.0','move 0.0 1.0',
         'move -1.570963 1.0','move 1.570963 1.0','move 0.0 1.0','move 0.0 1.0',
         'move 1.570963 1.0','move 0.0 1.0','move 0.0 1.0','move 0.0 1.0','move 0.0 1.0',
         'move 0.0 1.0','move 1.570963 1.0','move 0.0 1.0','move 0.0 1.0','move 0.0 1.0',
         'move 0.0 1.0','move 0.0 1.0','move 0.0 1.0','move 0.0 1.0','move 0.0 1.0','move 0.0 1.0',
         'move -1.570963 1.0','move 0.0 1.0','move 0.0 1.0','move 0.0 1.0','move 0.0 1.0',
         'move 0.0 1.0','move 0.0 1.0','move 0.0 1.0','move 0.0 1.0','move 0.0 1.0',
         'move -1.570963 1.0','move 0.0 1.0','move 0.0 1.0','move 0.0 1.0','move 0.0 1.0',
         'move 0.0 1.0','move 0.0 1.0','move -1.570963 1.0','move 0.0 1.0','move 0.0 1.0',
         'move 0.0 1.0','move 0.0 1.0','move 0.0 1.0','move 1.570963 1.0','move 0.0 1.0',
         'move 0.0 1.0','move 1.570963 1.0','move 0.0 1.0','move 0.0 1.0','move 0.0 1.0',
         'move 0.0 1.0','move 0.0 1.0','move 0.0 1.0','move -1.570963 1.0']

# Environment.
areaMap = ['LLLLLLLLLLLLLLLLLLLLL',
           'L...@LL.............L',
           'L....LL.............L',
           'L..LLLL..LLLLLLLLL..L',
           'L..LLLL..LLLLLLLLL..L',
           'L....LL..LL.........L',
           'L....LL..LL.........L',
           'LLL..LL..LLLLLLLLL..L',
           'LLL..LL..LLLLLLLLL..L',
           'L....LL..LL.........L',
           'L..LLLL..LL..LLLLLLLL',
           'L..LLLL..LL..LLLLLLLL',
           'L........LL.........L',
           'L........LL.........L',
           'LLLLLLLLLLLLLLLLLLLLL']
             
# Setup.
state = osHelp.State(areaMap)
rover_slam = osHelp.Slam()
first = True
trail = list()
beliefL = list()
plots = list()
number = 1
NOISE_MOVE = 0.01

# Execute SLAM and plot.
for move in moves:
    meas, found = state.generate_measurements()
    belief, mu = rover_slam.process_measurements(meas)
    truth = (state.bot.x - state._start_position['x'], state.bot.y - state._start_position['y'])
    
    locEst = list()
    for i in range(2, len(mu)-1):
        if i % 2 == 0:
            loc = (mu[i][0], mu[i+1][0])
            locEst.append(loc)
        
    if first:
        beliefL.append((belief[0], belief[1]))
        trail.append((state.bot.x, state.bot.y))
        beliefX, beliefY = zip(*beliefL)
        trailX, trailY = zip(*trail)
        foundX, foundY = zip(*found)
        locX, locY = zip(*locEst)
    
        fig = plt.figure()
        plt.xlim([-1,state.cols])
        plt.ylim([-state.rows,1])
        plt.plot(trailX, trailY, '--')
        
        beliefX = np.asarray(beliefX) + state._start_position['x']
        beliefY = np.asarray(beliefY) + state._start_position['y']
        locX = np.asarray(locX) + state._start_position['x']
        locY = np.asarray(locY) + state._start_position['y']
        
        plt.scatter(beliefX, beliefY, c='b')
        plt.scatter(locX, locY, c='m')
        plt.plot(state.bot.x, state.bot.y, c='r', marker=(3,0,degrees(state.bot.bearing)-90), markersize=15)
        plt.savefig('gifFolder/plot_%02d.png' % number)
        number += 1
        plt.close()
        first = False

    action = move.split()
    state.update_according_to(move)
    beliefL.append((belief[0], belief[1]))
    trail.append((state.bot.x, state.bot.y))
    beliefX, beliefY = zip(*beliefL)
    trailX, trailY = zip(*trail)
    foundX, foundY = zip(*found)
    locX, locY = zip(*locEst)

    beliefX = np.asarray(beliefX) + state._start_position['x']
    beliefY = np.asarray(beliefY) + state._start_position['y']
    locX = np.asarray(locX) + state._start_position['x']
    locY = np.asarray(locY) + state._start_position['y']
    
    plt.xlim([-1,state.cols])
    plt.ylim([-state.rows,1])
    plt.plot(trailX, trailY, '--')
    plt.scatter(beliefX, beliefY, c='b')
    plt.scatter(locX, locY, c='m')
    plt.plot(state.bot.x, state.bot.y, c='r', marker=(3,0,degrees(state.bot.bearing)-90), markersize=15)
    plt.savefig('gifFolder/plot_%02d.png' % number)
    number += 1
    plt.close()
    
    rover_slam.process_movement(float(action[1]), float(action[2]), NOISE_MOVE)

# Create gif.
plotList = []
plots = glob.glob('gifFolder/*.png')
for plot in plots:
    plotList.append(imageio.imread(plot))
imageio.mimsave('onlineSLAM.gif', plotList, fps=2)

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
roverSlam = osHelp.Slam()
first = True
trail = list()
beliefL = list()
plots = list()
fileNum = 1
moveNoise = 0.01

# Execute SLAM and plot.
for move in moves:
    meas, found = state.createMeasurements()
    belief, mu = roverSlam.processMeasurements(meas)
    truth = (state.bot.x - state.startPosition['x'], state.bot.y - state.startPosition['y'])
    
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

        beliefX = np.asarray(beliefX) + state.startPosition['x']
        beliefY = np.asarray(beliefY) + state.startPosition['y']
        locX = np.asarray(locX) + state.startPosition['x']
        locY = np.asarray(locY) + state.startPosition['y']
        
        # Plot.
        plt.xlim([-1,state.cols])
        plt.ylim([-state.rows,1])
        plt.plot(trailX, trailY, '--')
        plt.scatter(beliefX, beliefY, c='b')
        plt.scatter(locX, locY, c='m')
        plt.plot(state.bot.x, state.bot.y, c='r', marker=(3,0,degrees(state.bot.bearing)-90), markersize=15)
        plt.savefig('gifFolder/plot_%02d.png' % fileNum)
        fileNum += 1
        plt.close()
        first = False

    action = move.split()
    state.actionUpdate(move)
    beliefL.append((belief[0], belief[1]))
    trail.append((state.bot.x, state.bot.y))
    beliefX, beliefY = zip(*beliefL)
    trailX, trailY = zip(*trail)
    foundX, foundY = zip(*found)
    locX, locY = zip(*locEst)

    beliefX = np.asarray(beliefX) + state.startPosition['x']
    beliefY = np.asarray(beliefY) + state.startPosition['y']
    locX = np.asarray(locX) + state.startPosition['x']
    locY = np.asarray(locY) + state.startPosition['y']
    
    # Plot.
    plt.xlim([-1,state.cols])
    plt.ylim([-state.rows,1])
    plt.plot(trailX, trailY, '--')
    plt.scatter(beliefX, beliefY, c='b')
    plt.scatter(locX, locY, c='m')
    plt.plot(state.bot.x, state.bot.y, c='r', marker=(3,0,degrees(state.bot.bearing)-90), markersize=15)
    plt.savefig('gifFolder/plot_%02d.png' % fileNum)
    fileNum += 1
    plt.close()
    
    roverSlam.processMovement(float(action[1]), float(action[2]), moveNoise)

# Create gif.
plotList = []
plots = glob.glob('gifFolder/*.png')
for plot in plots:
    plotList.append(imageio.imread(plot))
for i in range(5):
    plotList.append(imageio.imread(plots[len(plots)-1]))
imageio.mimsave('onlineSLAM.gif', plotList, fps=2)

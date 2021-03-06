# -*- coding: utf-8 -*-
"""
Created on Sun Apr 26 22:08:06 2020

@author: David
"""
import numpy as np
from matplotlib import pyplot as plt

def aStarSearch(grid, init, goal, cost, delta):
    # Initialize.
    x = init[0]
    y = init[1]
    pathCost = 0
    front = [(pathCost, init)]
    expanded = []
    pathList = {}
    pathList[str((x, y))] = [init]
    wallsY, wallsX = np.where(grid)
    fileNum = 1
    search = True

    while search:
        if not front:
            return 'failed'
        else:
            front.sort()
            nextSpace = front.pop(0)

            x = nextSpace[1][0]
            y = nextSpace[1][1]
            pathCost = nextSpace[0]
            
            # Check if goal reached.
            if x == goal[0] and y == goal[1]:
                    search = False
            else:
                for i in range(len(delta)):
                    inFront = inExpand = False
                    dx = delta[i][0]
                    dy = delta[i][1]
                    newSpace = [x + dx, y + dy]
            
                    # Check if visited.
                    for f in front:
                        if f[1] == newSpace:
                            inFront = True
                            break
                    for e in expanded:
                        if e == newSpace:
                            inExpand = True
                            break
                    
                    if (newSpace[0] >= 0) and (newSpace[0] < len(grid)) and (
                            newSpace[1] >= 0) and (newSpace[1] < len(grid[0])) and (
                            inFront == False) and (inExpand == False) and (
                            grid[newSpace[0]][newSpace[1]] != 1):
                        
                        heuristic = np.linalg.norm(np.asarray(newSpace) - np.asarray(goal))
                        newPathCost = pathCost + cost + heuristic
                        front.append((newPathCost, newSpace))
                        
                        newPath = list(pathList[str((x, y))])
                        newPath.append([newSpace[0], newSpace[1]])
                        pathList[str((newSpace[0], newSpace[1]))] = newPath
                        
                        expanded.append(newSpace)
                        expandY, expandX = zip(*expanded)
                        
                        # Plot.
                        plt.xlim([-1, len(grid[0])])
                        plt.ylim([len(grid), -1])
                        plt.scatter(expandX, expandY, c='b', marker='x')
                        plt.scatter(wallsX, wallsY, c='k', marker='s', s=50)
                        plt.plot(init[1], init[0], c='g', marker='o', markersize=15)
                        plt.plot(goal[1], goal[0], c='m', marker='*', markersize=15)
                        plt.savefig('gifFolder/plot_%03d.png' % fileNum)
                        fileNum += 1
                        plt.close()
                            
    bestPath = pathList[str((x, y))]
    bestPathY, bestPathX = zip(*bestPath)
    
    # Plot.
    plt.xlim([-1, len(grid[0])])
    plt.ylim([len(grid), -1])
    plt.scatter(expandX, expandY, c='b', marker='x')
    plt.scatter(wallsX, wallsY, c='k', marker='s', s=50)
    plt.plot(bestPathX, bestPathY, '-', c='r')
    plt.plot(init[1], init[0], c='g', marker='o', markersize=15)
    plt.plot(goal[1], goal[0], c='m', marker='*', markersize=15)
    for i in range(20):
        plt.savefig('gifFolder/plot_%03d.png' % fileNum)
        fileNum += 1
    plt.close()
    
    return bestPath
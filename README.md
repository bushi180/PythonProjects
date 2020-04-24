# PythonProjects

# Table of Contents
   * [Requirements](#requirements)
   * [How to use](#how-to-use)
   * [Projects](#projects)
      * [Online SLAM](#online-slam)
	  * [A-Star (A*)](#a-star)
	  
# Requirements

- Python 2.7

- numpy

- matplotlib

- glob

- imageio

# How to use

1. Clone this repo.

> git clone https://github.com/bushi180/PythonProjects.git


2. Install the required libraries. You can use environment.yml with conda command.

> conda env create -f environment.yml


3. Execute python script in each directory.

# Projects

## Online SLAM

Online Simultaneous Localization and Mapping (SLAM)

Calculates the location of a bot and landmarks after a series of movements and measurements to landmarks.

- red triangle - actual bot position and direction

- blue dash line - actual bot path

- blue dots - SLAM calculated bot positions

- purple dots- SLAM calculated landmark positions

![2](https://github.com/bushi180/PythonProjects/blob/master/ProjectGifs/onlineSLAM.gif)

## A-Star (A*)

Calculates the shortest path found between two nodes. Uses an Euchildean distance heuristic.

- green circle - start position

- magenta star - goal postion

- black squares - walls

- blue X's - searched nodes

- red line - shortest path found

![2](https://github.com/bushi180/PythonProjects/blob/master/ProjectGifs/aStar.gif)
# PythonProjects

# Table of Contents
   * [Requirements](#requirements)
   * [How to use](#how-to-use)
   * [Projects](#projects)
      * [Online SLAM](#online-slam)
	  * [A-Star](#a-star)
	  * [Kalman Filter](#kalman-filter)
	  * [Colorization Using Optimization](#colorization-using-optimization)
	  
# Requirements

- Python 2.7

- numpy 1.15

- scipy 1.2.1

- matplotlib

- imageio 2.6.1

- imutils 0.5.3

- opencv 2.4.13

- PyQt 5.6

# How to use

1. Clone this repo.

> git clone https://github.com/bushi180/PythonProjects.git


2. Install the required libraries. You can use environment.yml with conda command.

> conda env create -f environment.yml


3. Execute python script in each directory.

# Projects

## Online SLAM

Online Simultaneous Localization and Mapping (SLAM). Calculates the location of a bot and landmarks after a series of 
bot movements and measurements to landmarks. Results in the mapping of the environment. Online version only maintains 
the most recent position in bot's path, preventing the omega matrix from becoming too large.

- red triangle - actual bot position and direction

- blue dash line - actual bot path

- blue dots - SLAM calculated bot positions

- purple dots- SLAM calculated landmark positions

![2](https://github.com/bushi180/PythonProjects/blob/master/ProjectImages/onlineSLAM.gif)

## A-Star

An informed search algorithm that calculates the shortest path found between two nodes or graphs. Used an Euchildean 
distance heuristic.

- green circle - start position

- magenta star - goal postion

- black squares - walls

- blue X's - searched nodes

- red line - shortest path found

![2](https://github.com/bushi180/PythonProjects/blob/master/ProjectImages/aStar.gif)

## Kalman Filter

An optimal estimation algorithm that predicts states from uncertain measurements. Used to calculate the position 
of people in frame. Used OpenCV Histogram of Gradients (HoG) descriptors as measurements to update the filter.

- green rectangle - bounding box created from kalman filter state after prediction and measurement update.

![2](https://github.com/bushi180/PythonProjects/blob/master/ProjectImages/kalmanFilter.gif)

## Colorization Using Optimization

Process is accomplished with the method introduced in the paper Colorization using Optimization
by Anat Levin, Dani Lischinski, and Yair Weiss. It is based on the premise that neighboring pixels in 
space-time that have similar intensities should have similar colors.

The modified PyQt Scribble interface allows a user to import a black and white image, apply color marks, 
and through colorization using optimization produce a fully colored image. There are options to draw new 
color marks on an image or import an image pre-colored with marks.

![2](https://github.com/bushi180/PythonProjects/blob/master/ProjectImages/colorOpGui.png)
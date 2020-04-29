# PythonProjects

# Table of Contents
   * [Requirements](#requirements)
   * [How to use](#how-to-use)
   * [Projects](#projects)
	  * [Colorization Using Optimization](#colorization-using-optimization)
	  * [Seam Carving For Content-Aware Image Resizing](#seam-carving-for-content-aware-image-resizing)
	  * [Stereo Correspondence](#stereo-correspondence)
	  * [Online SLAM](#online-slam)
	  * [A-Star](#a-star)
	  * [Kalman Filter](#kalman-filter)
	  
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

## Colorization Using Optimization

Process is accomplished with the method introduced in the paper [Colorization using Optimization](https://www.cs.huji.ac.il/~yweiss/Colorization/)
by Anat Levin, Dani Lischinski, and Yair Weiss. It is based on the premise that neighboring pixels in space-time 
that have similar intensities should have similar colors.

The modified PyQt Scribble interface allows a user to import a black and white image, apply color marks, 
and through colorization using optimization produce a fully colored image. There are options to draw new 
color marks on an image or import an image pre-colored with marks.

![Color Optimization](https://github.com/bushi180/PythonProjects/blob/master/ProjectImages/colorOpGui1.png)

## Seam Carving For Content-Aware Image Resizing

Process is accomplished with the method introduced in the paper [Seam Carving for Content-Aware Image Resizing](http://www.faculty.idc.ac.il/arik/SCWeb/imret/index.html) 
by Shai Avidan and Ariel Shamir. It is based on the premise that a seam is an optimal 8-connected path of pixels 
on a single image from top to bottom or left to right. By repeatedly carving out or inserting low cost 
seams, defined by an energy map, the aspect ratio of an image can be changed. The selection and order of seams protect 
the content of the image.

Image Reduction

![Seam Remove](https://github.com/bushi180/PythonProjects/blob/master/ProjectImages/seamRemoving.gif)

Image Expansion

![Seam Add](https://github.com/bushi180/PythonProjects/blob/master/ProjectImages/seamAdding.gif)

## Stereo Correspondence

Used a window based correlation method to determine which pixels in a left stereo image correspond with pixels in a 
right stereo image. To compare windows I used the sum of squared difference. The disparity map shows the difference in 
pixel coordinates of similar features in the left and right stereo images.

Rectified stereo images

![Seam Remove](https://github.com/bushi180/PythonProjects/blob/master/ProjectImages/mleft_right.jpg)

Left and right disparity (depth) maps.

![Seam Add](https://github.com/bushi180/PythonProjects/blob/master/ProjectImages/mDis_left_right)

Occluded areas in black.

![Seam Add](https://github.com/bushi180/PythonProjects/blob/master/ProjectImages/motorcycle_occludedD.png)

## Online SLAM

Online Simultaneous Localization and Mapping (SLAM). Calculates the location of a bot and landmarks after a series of 
bot movements and measurements to landmarks. Results in the mapping of the environment. Online version only maintains 
the most recent position in bot's path, preventing the omega matrix from becoming too large.

- red triangle - actual bot position and direction

- blue dash line - actual bot path

- blue dots - SLAM calculated bot positions

- purple dots- SLAM calculated landmark positions

![Online SLAM](https://github.com/bushi180/PythonProjects/blob/master/ProjectImages/onlineSLAM.gif)

## A-Star

An informed search algorithm that calculates the shortest path found between two nodes or graphs. Used an Euclidean 
distance heuristic.

- green circle - start position

- magenta star - goal position

- black squares - walls

- blue X's - searched nodes

- red line - shortest path found

![A Star](https://github.com/bushi180/PythonProjects/blob/master/ProjectImages/aStar.gif)

## Kalman Filter

An optimal estimation algorithm that predicts states from uncertain measurements. Used to calculate the position 
of people in frame. Used OpenCV Histogram of Gradients (HoG) descriptors as measurements to update the filter.

- green rectangle - bounding box created from kalman filter state after prediction and measurement update.

![Kalman Filter](https://github.com/bushi180/PythonProjects/blob/master/ProjectImages/kalmanFilter.gif)
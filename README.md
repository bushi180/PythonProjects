# PythonProjects

# Table of Contents
   * [Requirements](#requirements)
   * [How to use](#how-to-use)
   * [Projects](#projects)
      * [Online SLAM](#online-slam)
	  
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
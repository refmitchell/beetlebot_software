# Beetlebot Software
This repository contains all *remote* software for the Beetlebot platform. The
intent is that this software be run on a remote machine (e.g. laptop) to limit
the on-board processing. Packages which must be on-board will be put into a
different repo (todo).

## Usage
This software depends on ROS Noetic (Ubuntu 20.04 Focal Fossa); make sure you have installed ROS and set it up
before continuing.

Clone the repo, then copy the contents into your catkin workspace (~/catkin_ws)
and run catkin_make.

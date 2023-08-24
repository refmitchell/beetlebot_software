# Beetlebot Software
This repository contains all *remote* software for the Beetlebot platform. The
intent is that this software be run on a remote machine (e.g. laptop) to limit
the on-board processing.

The [bb_sensors](https://github.com/refmitchell/bb_sensors) package contains
the code which runs on the robot.

## Usage
This code is built for ROS1 Noetic which itself depends on Ubuntu 20.04.
You must be using an Ubuntu 20.04 system with ROS1 Noetic installed.

Clone this repository into your catkin workspace (along with the dependencies
listed below) and run catkin_make as usual. 

### Dependencies
#### Compile-time
- [vision_opencv](https://github.com/ros-perception/vision_opencv) (make sure you download the version for ROS Noetic).

Clone this into your catkin workspace

####  Run-time dependencies
This code is designed to work with a TurtleBot3 (Burger) which requires the
following ROS packages. These can be installed on the system but I found it more
convenient to include these in my catkin workspace and build them from source.

- [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
- [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)

**Note** that each of these repos has a number of branches
  available. Make sure you use the ROS Noetic version.

#### Dependency repository
For convenience, I stored a snapshot of all
of the above dependencies which is available
[here](https://github.com/refmitchell/beetlebot_dependencies). If you
clone the dependency repo into your catkin workspace then the versions
should all be compatible. There is no guarantee that the code in the
dependency repository is up-to-date.

## Documentation
Pre-built documentation is provided per package using rosdoc_lite. Within
each package, go to doc/html/index.html to view the package documentation.
This documentation also provides instructions for running each available
ROS node and describes how the different nodes fit together.

For each package, the documentation documentation can be re-built using:

```
$ cd bb_<package>
$ rosdoc_lite .
```

## Included work

This repository includes some work which is not my own. Specifically, within
the bb_util package, the
[Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) linear
algebra library and the
[argparse](https://github.com/jamolnng/argparse/tree/develop) header
library.

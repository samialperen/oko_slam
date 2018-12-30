# oko_slam
**NOTE:** The following people contributed equally to this project:
* Oğuz Özdemir - https://github.com/ozzdemir
* Uğur Açıkgöz - https://github.com/ugurmozart
* Sami Alperen Akgün - https://github.com/samialperen
* Mustafa Kılınç - https://github.com/dorasprime
* Furkan Güllü - https://github.com/gullufurkan

This repository contains all the documentation, code and design of the robot made by "Öz Kardeşler Otomasyon (OKO)".
The main purpose of the robot is to extract map of the unknown indoor environment. It is an alternative and cheap platform to study on Simultaneous Localization and Mapping (SLAM).

## Prerequisites
* ROS Kinetic
* Ubuntu 16.04
* [ros-kinetic-turtlebot3 (already installed in ros_ws)](https://github.com/ROBOTIS-GIT/turtlebot3)
* [ros-kinetic-hector_mapping and hector_slam (already installed in ros_ws)](https://github.com/tu-darmstadt-ros-pkg/hector_slam)
## Installation 
Go to directory ros_ws.
```
$ cd oko_slam/ros_ws
```
Build ROS workspace with catkin
```
$ catkin_make
```
**NOTE:** Do not forget to make source to setup.bash after build. You have to source setup.bash from each new terminal. If you don't want to do this, you can make source in your .bashrc file.
```
$ source ros_ws/devel/setup.bash
```
## Documentation
For documentation and tutorials please see the doc directory.

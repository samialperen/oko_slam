#!/bin/bash

# write your ip address of the raspberry pi here
export ROS_HOSTNAME=10.42.0.1
export ROS_MASTER_URI=http://10.42.0.1:11311

# Gmapping 
source /opt/ros/kinetic/setup.bash
source /home/ozkardesler/Documents/oko_slam/ros_ws/devel/setup.bash

#roslaunch kamu_robotu_launch oko_slam.launch 



 

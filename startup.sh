#!/bin/bash

# write your ip address of the raspberry pi here
export ROS_HOSTNAME=10.42.0.1
export ROS_MASTER_URI=http://10.42.0.1:11311

 
source /opt/ros/kinetic/setup.bash
source /home/ozkardesler/Documents/oko_slam/ros_ws/devel/setup.bash
source /home/ozkardesler/cartog_ws/devel_isolated/setup.bash

currentdate=$(date +"%Y-%m-%d")
#Cartographer 

mkdir /home/ozkardesler/Documents/oko_slam/ros_ws/saved_maps/current_results/carto-$currentdate 
index = $(cat /home/ozkardesler/Documents/oko_slam/ros_ws/saved_maps/current_results/index.txt)
$(index++);
# Gmapping
#roslaunch kamu_robotu_launch oko_slam.launch 

# cartographer

roslaunch kamu_robotu_launch oko_slam.launch slam_type:=cartographer sim_time:=false

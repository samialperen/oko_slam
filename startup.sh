#!/bin/bash

# write your ip address of the raspberry pi here
export ROS_HOSTNAME=10.42.0.1
export ROS_MASTER_URI=http://10.42.0.1:11311

 
source /opt/ros/kinetic/setup.bash
source /home/ozkardesler/Documents/oko_slam/ros_ws/devel/setup.bash
source /home/ozkardesler/cartog_ws/devel_isolated/setup.bash

currentdate=$(date +"%Y-%m-%d")
#Cartographer 

index=$(cat /home/ozkardesler/Documents/oko_slam/ros_ws/saved_maps/current_results/index.txt)
echo $((index+1)) > /home/ozkardesler/Documents/oko_slam/ros_ws/saved_maps/current_results/index.txt;
mkdir /home/ozkardesler/Documents/oko_slam/ros_ws/saved_maps/current_results/carto-$currentdate
mkdir /home/ozkardesler/Documents/oko_slam/ros_ws/saved_maps/current_results/carto-$currentdate/test-$index
# Gmapping
#roslaunch kamu_robotu_launch oko_slam.launch 

# cartographer

roslaunch kamu_robotu_launch oko.launch slam_type:=cartographer sim_time:=false bagname:=carto-$currentdate/test-$index/test-$index.bag

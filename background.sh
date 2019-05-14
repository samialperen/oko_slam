#!/bin/bash
#	Author: Mustafa KILINÇ
#	E-mail: mustafa.kilinc@ieee.metu.edu.tr

# reconnect to the wifi 
#nmcli d connect wlan0

# Get your current ip address
sleep 1
  
  hostname -I >> ~/RaspiShare/ip.txt
  iwgetid  >> ~/RaspiShare/ssid.txt

echo  'kamurobotu' | sudo -S  rclone copy -v ~/RaspiShare/ ozkardesler: 

# you can also sync you shared file with the command below  
#echo  'kamurobotu' | sudo -S   rclone sync ozkardesler: ~/RaspiShare



# Source ROS_WORKSPACE

source /opt/ros/kinetic/setup.bash
source /home/ozkardesler/Documents/oko_slam/ros_ws/devel/setup.bash
source /home/ozkardesler/cartog_ws/devel_isolated/setup.bash



# ROS MASTER ip update
export ROS_HOSTNAME=10.42.0.1
export ROS_MASTER_URI=http://10.42.0.1:11311



# How to get ip from txt file for ROS_MASTER_URI 

#file="/home/ozkardesler/RaspiShare/ip.txt"
#sed -i 's/[[:space:]]*$//' $file
#tag=$( tail -n 1 $file  )
#export ROS_MASTER_URI=http://$tag:11311
#export ROS_HOSTNAME=$tag

# Take a picture of the current updated slam result 
# get date and time for the file name 
currentdate=$(date +"%Y-%m-%d")
currenttime=$(date +"%T")

#Cartographer 
index=$(cat /home/ozkardesler/Documents/oko_slam/ros_ws/saved_maps/current_results/index.txt)


mkdir /home/ozkardesler/Documents/oko_slam/ros_ws/saved_maps/current_results/carto-$currentdate
mkdir /home/ozkardesler/Documents/oko_slam/ros_ws/saved_maps/current_results/carto-$currentdate/test-$index

echo  'kamurobotu' | sudo -S  rclone copy -v ~/Documents/oko_slam/ros_ws/saved_maps/ ozkardesler:
rosrun carto_map_server carto_map_saver -f /home/ozkardesler/Documents/oko_slam/ros_ws/saved_maps/current_results/carto-$currentdate/test-$index/$currenttime
rosrun kamu_robotu_map_analyzer map_analyzer /home/ozkardesler/Documents/oko_slam/ros_ws/saved_maps/current_results/carto-$currentdate/test-$index/$currenttime
convert /home/ozkardesler/Documents/oko_slam/ros_ws/saved_maps/current_results/carto-$currentdate/test-$index/$currenttime.pgm  /home/ozkardesler/Documents/oko_slam/ros_ws/saved_maps/current_results/carto-$currentdate/test-$index/$currenttime.png
#rm /home/ozkardesler/Documents/oko_slam/ros_ws/saved_maps/current_results/carto-$currentdate/test-$index/$currenttime.pgm

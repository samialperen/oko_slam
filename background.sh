#!/bin/bash


#nmcli d connect wlan0

sleep 1
  
  hostname -I >> ~/RaspiShare/ip.txt
  iwgetid  >> ~/RaspiShare/ssid.txt

echo  'kamurobotu' | sudo -S  rclone copy -v ~/RaspiShare/ ozkardesler: 

 

#echo  'kamurobotu' | sudo -S   rclone sync ozkardesler: ~/RaspiShare

sleep 4

echo  'kamurobotu' | sudo -S  rclone copy -v ~/Documents/oko_slam/ros_ws/saved_maps ozkardesler:


#file="/home/ozkardesler/RaspiShare/ip.txt"

#sed -i 's/[[:space:]]*$//' $file


#tag=$( tail -n 1 $file  )

#export ROS_MASTER_URI=http://$tag:11311
#export ROS_HOSTNAME=$tag

export ROS_HOSTNAME=10.42.0.1
export ROS_MASTER_URI=http://10.42.0.1:11311


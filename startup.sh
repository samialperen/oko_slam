#!/bin/sh


sleep 10

#nmcli d connect wlan0

sleep 3
  
  hostname -I >> ~/RaspiShare/ip.txt
  iwgetid  >> ~/RaspiShare/ssid.txt

echo  'kamurobotu' | sudo -S  rclone copy -v ~/RaspiShare/ ozkardesler: 

 

#echo  'kamurobotu' | sudo -S   rclone sync ozkardesler: ~/RaspiShare

sleep 4

file="/home/ozkardesler/RaspiShare/ip.txt"

sed -i 's/[[:space:]]*$//' $file


tag=$( tail -n 1 $file  )

export ROS_MASTER_URI=http://$tag:11311
export ROS_HOSTNAME=$tag

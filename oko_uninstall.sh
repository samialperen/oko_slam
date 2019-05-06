#!/bin/bash
#lamp

echo "Oz Kardesler Otomasyon"
echo "Uninstallation Script for Raspberry-Pi/General Computer"
echo "Please Enter Input:{raspberry, pc} "
read input

if [ $input == "raspberry" ]; then
	sudo apt update
	
    echo "#####################"
    echo "removing ROS Serial"
	sudo apt-get remove ros-kinetic-serial -y
	
    echo "#####################"
	echo "removing Gmapping"
	sudo apt-get remove ros-kinetic-gmapping -y

	echo "#####################"	
	echo "removing Hector-SLAM"
	sudo apt-get remove ros-kinetic-hector-slam -y
	
		
elif [ $input == "pc" ]; then
	sudo apt update
	
	echo "#####################"
    echo "removing ROS Serial"
	sudo apt-get remove ros-kinetic-serial -y

	echo "#####################"	
	echo "removing Gmapping"
	sudo apt-get remove ros-kinetic-gmapping -y 

	echo "#####################"	
	echo "removing Hector-SLAM"
	sudo apt-get remove ros-kinetic-hector-slam -y

	echo "#####################"	
	echo "removing ROS Navigation Stack"
	sudo apt-get remove ros-kinetic-navigation -y

	echo "#####################"	
	echo "removing Turtlebot3"
	sudo apt-get remove ros-kinetic-turtlebot3 -y

	echo "#####################"	
	echo "removing Frontier Exploration"
	sudo apt-get remove ros-kinetic-frontier-exploration -y

	echo "#####################"	
	echo "removing ROS Navigation-Stage"
	sudo apt-get remove ros-kinetic-navigation-stage -y

	echo "#####################"	
	echo "removing Explore-Lite"
	sudo apt-get remove ros-kinetic-explore-lite -y 
		
else 
	echo "Nothing was removed!"
	echo "You need to enter input: raspberry or pc"
fi


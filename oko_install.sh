#!/bin/bash
#lamp

echo "Oz Kardesler Otomasyon"
echo "Installation Script for Raspberry-Pi/General Computer"
echo "Please Enter Input:{raspberry, pc} "
read input

if [ $input == "raspberry" ]; then
	sudo apt update
	
    echo "#####################"
    echo "Installing ROS Serial"
	sudo apt-get install ros-kinetic-serial -y
	
    echo "#####################"
	echo "Installing Gmapping"
	sudo apt-get install ros-kinetic-gmapping -y

	echo "#####################"	
	echo "Installing Hector-SLAM"
	sudo apt-get install ros-kinetic-hector-slam -y
	
		
elif [ $input == "pc" ]; then
	sudo apt update
	
	echo "#####################"
    echo "Installing ROS Serial"
	sudo apt-get install ros-kinetic-serial -y

	echo "#####################"	
	echo "Installing Gmapping"
	sudo apt-get install ros-kinetic-gmapping -y 

	echo "#####################"	
	echo "Installing Hector-SLAM"
	sudo apt-get install ros-kinetic-hector-slam -y

	echo "#####################"	
	echo "Installing ROS Navigation Stack"
	sudo apt-get install ros-kinetic-navigation -y

	echo "#####################"	
	echo "Installing Turtlebot3"
	sudo apt-get install ros-kinetic-turtlebot3 -y

	echo "#####################"	
	echo "Installing Frontier Exploration"
	sudo apt-get install ros-kinetic-frontier-exploration -y

	echo "#####################"	
	echo "Installing ROS Navigation-Stage"
	sudo apt-get install ros-kinetic-navigation-stage -y

	echo "#####################"	
	echo "Installing Explore-Lite"
	sudo apt-get install ros-kinetic-explore-lite -y 
		
else 
	echo "Nothing was installed!"
	echo "You need to enter input: raspberry or pc"
fi

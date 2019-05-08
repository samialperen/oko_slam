# USER MANUAL of KAMU ROBOTU
**NOTE:** The following people contributed equally to this project:

* Oğuz Özdemir - https://github.com/ozzdemir
* Uğur Açıkgöz - https://github.com/ugurmozart
* Sami Alperen Akgün - https://github.com/samialperen
* Furkan Güllü - https://github.com/gullufurkan
* Mustafa Kılınç - https://github.com/dorasprime

This repository contains all the documentation, code and design of the robot made by "Öz Kardeşler Otomasyon (OKO)".
The main purpose of the robot is to extract map of the unknown indoor environment. It is an alternative and cheap platform to study on Simultaneous Localization and Mapping (SLAM).

## Prerequisites
* ROS Kinetic
* Ubuntu 16.04

## Installation 
Run oko_install.sh to install necessary ros packages using apt manager. If you want to use this meta-package in raspberry pi enter input as raspberry. Otherwise, enter input as pc.
```
$ cd oko_slam
$ . oko_install.sh
Oz Kardesler Otomasyon
Installation Script for Raspberry-Pi/General Computer
Please Enter Input:{raspberry, pc} 
$ raspberry
```

Now install source packages inside of oko_slam ROS meta-package.
Go to directory ros_ws:
```
$ cd oko_slam/ros_ws
```

Build ROS workspace with catkin
```
$ catkin_make
```

**NOTE:** Do not forget to make source to setup.bash after build. You have to source setup.bash from each new terminal. If you don't want to do this, you can add below source line into .bashrc file.
```
$ source ros_ws/devel/setup.bash
```

If you want to install and use **Cartographer_ROS** SLAM algorithm, then you need to follow [Cartographer_ROS SLAM INSTALLATION GUIDE & TUTORIALS](/doc/cartographer_slam_tutorial.md)

## Documentation
For documentation and tutorials please follow the next sections.

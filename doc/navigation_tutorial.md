# ROS NAVIGATION STACK + EXPLORATION PACKAGES INSTALLATION GUIDE & TUTORIALS

* Before this tutorial, please visit [HECTOR SLAM INSTALLATION GUIDE & TUTORIALS](/doc/hector_slam_tutorial.md)
* Before this tutorial, please visit [GMAPPING INSTALLATION GUIDE & TUTORIALS](/doc/gmapping_tutorial.md)
* Before this tutorial, please visit [Cartographer_ROS SLAM INSTALLATION GUIDE & TUTORIALS](/doc/cartographer_slam_tutorial.md)

[ROS Navigation Stack](http://wiki.ros.org/navigation) takes in information from odometry, sensor streams, and a goal pose and outputs safe velocity commands that are sent to a mobile base. In short, it is used to navigate the robot. You can use ROS Navigation Stack with some additional exploration packages to navigate autonomously while extracting the map itself at the same time. There is also another option that you can use saved map of the environment (static map) to navigate autonomously. 

[Frontier Exploration](http://wiki.ros.org/frontier_exploration) is an exploration package which gives automatic movement commands to ROS Navigation stack in order to navigate autonomously in a map whose borders is chosen at the beginning.

[Explore Lite](http://wiki.ros.org/explore_lite) is an alternative exploration package which provides greedy frontier-based exploration. This package works in the same way with frontier exploration. However, it does not need any boundary to be given at the beginning in order to run. 

## Prerequisites
* Ubuntu 16.04
* ROS Kinetic
* Gmapping
* Gazebo

## Installation

### Navigation Stack
Install ROS Navigation stack using apt package manager.
```
$ sudo apt-get install ros-kinetic-navigation
```

### Frontier Exploration
Install frontier exploration package.
```
$ sudo apt-get install ros-kinetic-frontier-exploration
$ sudo apt-get install ros-kinetic-navigation-stage
```

### Explore-lite
```
$ sudo apt-get install ros-kinetic-explore-lite
```

## Tutorials

### Tutorial 1: Run ROS Navigation Package 
As mentioned in [Robot Setup Tutorial of navigation](http://wiki.ros.org/navigation/Tutorials/RobotSetup), your robot should have a sensor source (LIDAR in our case), odometry unit and base controller which subscribes **/cmd_vel** ROS topic to get movement commands. 

**Move Base** node has two planners: global and local respectively. Global planner tries to achieve the desired goal while local planner provides obstacle avoidance.

According to [Robot Setup Tutorial of navigation](http://wiki.ros.org/navigation/Tutorials/RobotSetup) we created our own [move_base.launch](/ros_ws/src/kamu_robotu/kamu_robotu_navigation/launch/move_base.launch) with necessary [parameters](/ros_ws/src/kamu_robotu/kamu_robotu_navigation/params).

Please realize that this launch and parameters are specific to our robot. If you want to use **move_base** for another robot, you need to change the parameters accordingly like in [Robot Setup Tutorial of navigation](http://wiki.ros.org/navigation/Tutorials/RobotSetup).

* First start our simulation environment.
```
$ roslaunch kamu_robotu_gazebo kamu_map.launch
```

* Start SLAM process (gmapping).
```
$ roslaunch kamu_robotu_launch oko_slam.launch sim_time:=true slam_type:=gmapping
```

You should be able to see a map in Rviz like below:
![Gmapping Rviz Output](/doc/images/navigation_tutorials/1.png)

* Start move_base node which takes goals to navigate the robot
to a desired point
```
$ roslaunch kamu_robotu_navigation move_base.launch
```

Click **2D Nav Goal** at the top of Rviz window:
![2D Nav Goal Button](/doc/images/navigation_tutorials/2.png)

Then select a path with your mouse:
![2D Nav Goal Path](/doc/images/navigation_tutorials/3.png)

Now you should be able to see the robot moving towards the path you selected. If this is the case, then it means we successfully completed this tutorial. If your robot does not move, start to debug the problem by looking at the errors/warnings in the terminal where you launched move_base. 

### Tutorial 2: Run Frontier Exploration
You need to first follow Tutorial 1. What frontier exploration does is creating some goals to sent the move_base node. Since it is practically does not make any sense to select navigation goals each time in Rviz like in the Tutorial 1, frontier exploration provides this automatically.

* Run frontier_exploration which creates goals for move_base automatically to navigate autonomously
```
$ roslaunch kamu_robotu_navigation frontier.launch
```

* In Rviz window, add **Marker** and choose marker topic: **/exploration_polygon_marker**
![Add Marker Exploration Polygon](/doc/images/navigation_tutorials/4.png)

* Click publish point at the top of rviz window and 
select a point
![Click Publish Point](/doc/images/navigation_tutorials/5.png)

* Repeat above step to create a closed polygon.
![Close the polygon](/doc/images/navigation_tutorials/6.png)

* After your closed polygon becomes red color, click 
publish point button again to give an initial exploration point
in closed polygon to the robot.
![Give an initial exploration point](/doc/images/navigation_tutorials/7.png)
```

After all these steps, robot will autonomously navigate and try to 
extract map of the overall environment inside the given polygon. 

### Tutorial 3: Run Explore Lite Instead of Frontier-Exploration
Explore lite is a greedy frontier based exploration algorithm and it can be used instead of frontier-based exploration ROS package. 

* Follow Tutorial 1 and after launching move_base, launch explore_lite instead
of frontier_exploration
```
$ roslaunch explore_lite explore.launch
```

### Tutorial 4: Run Our Own Autonomous Navigation Package


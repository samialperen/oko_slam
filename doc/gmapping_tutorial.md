# GMAPPING INSTALLATION GUIDE & TUTORIALS

Before this tutorial, please visit [HECTOR SLAM INSTALLATION GUIDE & TUTORIALS](/doc/hector_slam_tutorial.md)

[Gmapping](http://wiki.ros.org/gmapping) is a mapping algorithm which uses laser scan and odometry information to extract a map of the environment.

## Prerequisites
* Ubuntu 16.04
* ROS Kinetic
* Turtlebot3 ROS Kinetic Package (optional)
* Gazebo

## Installation
Install gmapping using apt package manager.
```
$ sudo apt-get install ros-kinetic-gmapping
```

**NOTE:** Gmapping is compatible with only ROS kinetic, indigo and lunar distributions according to the [gmapping ROS wiki](http://wiki.ros.org/gmapping)

That's all for the installation. You can move on the next section.

## Tutorials

### Tutorial 1: Run Gmapping with Turlebot3 in Gazebo
For more information please visit the [reference source](http://emanual.robotis.com/docs/en/platform/turtlebot3/slam/).

Specify model of the turtlebot3 you are using. Since we are using Gazebo, model name is not so important. Just pick one of the models.
```
$ export TURTLEBOT3_MODEL=waffle_pi
```
**NOTE:** You have to export each time from each terminal before calling the necessary ros functions. If you do not want to do this, you can add above command to .bashrc file.

Open Gazebo environment with specified world environment.
```
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Launch the gmapping.
```
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

Move turtlebot 3 with keyboard.
```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

After obtained full map you can save it to anywhere you like.
```
$ rosrun map_server map_saver -f ~/map1
```

### Tutorial 2: Move Turtlebot3 with Recorded Rosbag instead of Teleoperation
Generally people want to test gmapping with different parameters and for these situations using teleoperation to move turtlebot 3 does not make any sense since you need to move in same way for all cases to make a fair comparison. To solve this, one can move turtlebot3 with teleop just one time and record the movement of turtlebot3 using rosbag. Then, this rosbag file can be used later to move turtlebot3 in the same way.
 
Open Gazebo environment with specified world environment
```
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Launch the gmapping
```
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

Go to our example [rosbag file](/ros_ws/bagfiles/simulation_data/turtlebot3_movement.bag) that keeps movement commands to turtlebot3.
```
$ cd oko_slam/ros_ws/bagfiles
```

Play our bagfile.
```
$ rosbag play turtlebot3_movement.bag
```

**NOTE:** We used waffle_pi as TURTLEBOT3_MODEL. It is highly possible that this bag file is specific to waffle_pi. If you are using another model, you need to save your own bag file.

**NOTE:** If you want to change the parameters of the gmapping:
```
$ roscd turtlebot3_slam
$ cd launch
$ gedit turtlebot3_gmapping.launch
```
In here you can adjust gmapping parameters as you wish. You can see [this tutorial](http://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#tuning-guide) to understand how to tune gmapping for your application.

### Tutorial 3: Run gmapping with your own Data and without Gazebo
For more information please see [slam_gmapping tutorial](http://wiki.ros.org/cn/slam_gmapping/Tutorials/MappingFromLoggedData).

Run gmapping node with necessary parameters to subscribe /odom and /scan topics.
```
$ rosrun gmapping slam_gmapping scan:=scan odom_frame:=odom use_sim_time:=true 
```

Run the bagfile you record. You might record this bagfile during either in simulation or real world. For example, we recorded [a bag file](/ros_ws/bagfiles/real_data/realmap3objects-linearvel30-lidar-4hz2.bag) which contains only laser scan data in /scan topic. We used Turtlebot3 in Gazebo to record this bag file. However, you can use any bagfile you want.
```
$ cd oko_slam/ros_ws/bagfiles/real_data
$ rosbag play realmap3objects-linearvel30-lidar-4hz2.bag --clock
```

**NOTE:** Please note that this bag file publishes laser scan data to /scan , odom data to /odom and tf tree to /tf topic. Gmapping subscribes /scan and /tf topics.

**NOTE:** Generally it is common practice to create a launch file to call gmapping node with appropriate parameters since it is difficult to call gmapping from the terminal with lots of parameters. For this purpose, you can check [oko_gmapping.launch](/ros_ws/src/kamu_robotu/kamu_robotu_launch/launch/oko_gmapping.launch) file and use it as a reference for your application.

If everything is okay, you should be able to see Rviz output like below:
![Gmapping Result for realmap3objects-linearvel30-lidar-4hz2.bag](/doc/images/gmapping_tutorials/1.png)
<center> Gmapping Result for realmap3objects-linearvel30-lidar-4hz2.bag </center>

**Important NOTE:** Gmapping package needs specific transform tree(tf) configuration to work properly. If you can not see any output, you may need to adjust tf tree or change the parameters. In our case, required transformations are done by robot_state_publisher in [oko_gmapping.launch](/ros_ws/src/kamu_robotu/kamu_robotu_launch/launch/oko_gmapping.launch) file and some of them are included already in odom message.
 
Let's launch gmapping with our launch file. Our oko_slam.launch file calls oko_gmapping.launch file with some arguments.
```
$ roslaunch kamu_robotu_launch oko_slam.launch sim_time:=true slam_type:=gmapping
```

Then play the rosbag file again.

Correct TF tree configuration for our case can be seen below:
![Gmapping Transform Tree Configuration](/doc/images/gmapping_tutorials/2.png)
<center> Gmapping Transform Tree Configuration </center>

**Important NOTE:** You can also use our launcher with a real data without saving as a rosbag file. LIDAR should publish to /scan topic by default, but it can be changed by modifying launch files mentioned before. However, one should set **/use_sim_time** parameter as **false** while **using real time data**. On the other hand, it should be set **true** to **use with rosbag** files. 

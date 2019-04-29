# HECTOR SLAM INSTALLATION GUIDE & TUTORIALS

* [Hector SLAM](http://wiki.ros.org/hector_slam) is a mapping algorithm which only uses laser scan information to extract the map of the environment.

## Prerequisites
* Ubuntu 16.04
* ROS Kinetic

## Installation
First install turtlebot3 ROS package.
```
$ sudo apt-get install ros-kinetic-turtlebot3
```

Then, install hector-slam using apt package manager.
```
$ sudo apt-get install ros-kinetic-hector-slam
```

**NOTE:** Hector_slam is compatible with only ROS kinetic and indigo distributions according to the [hector_slam ROS wiki](http://wiki.ros.org/hector_slam)

That's all for the installation. You can move on the next section.

## Tutorials

### Tutorial 1: Run Hector SLAM with Turlebot3 in Gazebo
For more information please visit the [reference source](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#ros-packages-for-gazebo). 

Specify model of the turtlebot 3 you are using. Since we are using Gazebo, model name is not so important. Just pick one of the models.
```
$ export TURTLEBOT3_MODEL=waffle_pi
```
**NOTE:** You have to export each time from each terminal before calling the necessary ros functions. If you do not want to do this, you can add above command to .bashrc file.

Open Gazebo environment with specified world environment.
```
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Launch the hector_slam.
```
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=hector
```

Move turtlebot 3 with keyboard.
```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Now you should be able to see the map in Rviz like below:

![Turtlebot3 World Gazebo](/doc//images/hector_slam_tutorials/1.png)
<center> Turtlebot3 World Gazebo Environment </center>

![Turtlebot3 Hector SLAM Output in Rviz](/doc/images/hector_slam_tutorials/2.png)
<center> Turtlebot3 Hector SLAM Output in Rviz </center>

After obtained full map you can save it to anywhere you like.
```
$ rosrun map_server map_saver -f ~/map1
```

### Tutorial 2: Move Turtlebot3 with Recorded Rosbag instead of Teleoperation
Generally people want to test hector slam with different parameters and for these situations using teleoperation to move turtlebot 3 does not make any sense since you need to move in same way for all cases to make a fair comparison. To solve this, one can move turtlebot3 with teleop just one time and record the movement of turtlebot3 using rosbag. Then, this rosbag file can be used later to move turtlebot3 in the same way.
 
Open Gazebo environment with specified world environment
```
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Launch the hector_slam 
```
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=hector
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

### Tutorial 3: Customize LIDAR Parameters in Gazebo
If you are designing a new LIDAR or you have a LIDAR different than the one on Turtlebot3, then you may need to manipulate LIDAR parameters in simulation. This task is very easy thanks to so called **xacro** files. As a start point, you can start to change xacro files of Turtlebot3 itself. 

Go to turtlebot3_description directory
```
$ roscd turtlebot3_description
```

Go to urdf directory and open turtlebot3_waffle_pi.gazebo.xacro (if you use different model change it accordingly) 
```
$ cd urdf & gedit turtlebot3_waffle_pi.gazebo.xacro
```

**NOTE:** It is a good idea to copy original xacro file if you need to use default values later.

Change LIDAR parameters as you wish like below:
![Turtlebot3 LIDAR Parameters](/doc/images/hector_slam_tutorials/3.png)
<center> Turtlebot3 LIDAR Parameters </center>

You can also see [our robot's xacro file](/ros_ws/src/kamu_robotu/kamu_robotu_gazebo/urdf/kamu_robotu.xacro).

### Tutorial 4: Run Hector SLAM with your own Data and without Gazebo
For more information please see [reference source](https://www.youtube.com/watch?v=3C_eRtSoU78).

Open tutorial.launch provided by hector_slam package.
```
$ roscd hector_slam_launch
$ cd launch
$ gedit tutorial.launch
```

This launch file calls mapping_default.launch. Open it as well.
```
$ roscd hector_mapping
$ cd launch
$ gedit mapping_default.launch
```

Analyze these two launch files. You can change necessary parameters in these files for your application. For example, you may want to increase/decrease map_resolution etc. These parameters decide SLAM performance.

First, let's try to call the default launch file tutorial.launch
```
$ roslaunch hector_slam_launch tutorial.launch
```

Run the bagfile you record. You might record this bagfile during either in simulation or real world. For example, we recorded [a bag file](/ros_ws/bagfiles/simulation_data/turtlebot3_scan2.bag) which contains only laser scan data in /scan topic. We used Turtlebot3 in Gazebo to record this bag file. However, you can use any bagfile you want.
```
$ cd oko_slam/ros_ws/bagfiles/simulation_data
$ rosbag play turtlebot3_scan2.bag --clock
```

**NOTE:** Please note that this bag file only publishes laser scan data to /scan topic. We do not use any odometry information since hector slam only subscribes laser scan data.

If everything is okay, you should be able to see Rviz output like below:
![Hector SLAM Output for Turtlebot3_scan2.bag](/doc/images/hector_slam_tutorials/4.png)
<center> Hector SLAM Output for Turtlebot3_scan2.bag </center>

**Important NOTE:** Hector_slam package needs specific transform tree(tf) configuration to work properly. If you can not see any output in rviz like above, you may need to adjust tf tree. In our case, required transformations are done by robot_state_publisher in launch files.
You can check our [launch file oko_slam.launch](/ros_ws/src/kamu_robotu/kamu_robotu_launch/launch/oko_slam.launch) and [launch file oko_hector_mapping.launch](/ros_ws/src/kamu_robotu/kamu_robotu_launch/launch/oko_hector_mapping.launch). 

For more information about tf tree configuration for hector slam, one can check [official hector_slam tutorial](http://wiki.ros.org/hector_slam/Tutorials/SettingUpForYourRobot).

Let's launch hector_slam with our launch file. Our oko_slam.launch file calls oko_hector_mapping.launch file with some arguments.
```
$ roslaunch kamu_robotu_launch oko_slam.launch sim_time:=true slam_type:=hector_mapping
```

Then play the rosbag file again.

Correct TF tree configuration for our case can be seen below:
![Hector SLAM Transform Tree Configuration](/doc/images/hector_slam_tutorials/5.png)
<center> Hector SLAM Transform Tree Configuration </center>

**Important NOTE:** You can also use our launcher with a real data without saving as a rosbag file. LIDAR should publish to /scan topic by default, but it can be changed by modifying launch files mentioned before. However, one should set **/use_sim_time** parameter as **false** while **using real time data**. On the other hand, it should be set **true** to **use with rosbag** files. 

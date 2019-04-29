# HECTOR SLAM TUTORIALS

### Tutorial 1: Run Hector SLAM with Turlebot3 in Gazebo
For more information please visit the [reference source](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#ros-packages-for-gazebo). 

* Specify model of the turtlebot 3 you are using. Since we are using Gazebo, model name is not so important. Just pick one of the models.
```
$ export TURTLEBOT3_MODEL=waffle_pi
```
**NOTE:** You have to export each time from each terminal before calling the necessary ros functions. If you do not want to do this, you can add above command to .bashrc file.

* Open Gazebo environment with specified world environment.
```
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

* Launch the hector_slam.
```
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=hector
```

* Move turtlebot 3 with keyboard.
```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

* Now you should be able to see the map in Rviz like below.

![Turtlebot3 World Gazebo](/images/hector_slam_tutorials/1.png)
<center> Turtlebot3 World Gazebo Environment </center>
![Turtlebot3 Hector SLAM Output in Rviz](/images/hector_slam_tutorials/2.png)
<center> Turtlebot3 Hector SLAM Output in Rviz </center>

* After obtained full map you can save it to anywhere you like.
```
$ rosrun map_server map_saver -f ~/map1
```





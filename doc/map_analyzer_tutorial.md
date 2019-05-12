# MAP_ANALYZER INSTALLATION GUIDE & TUTORIALS

## Installation
### Installation Using Apt Manager (Recommended)
I followed two different source which are [stack-overflow](https://stackoverflow.com/questions/44060544/how-can-i-install-opencv-python-or-c-in-ubuntu/44060753#44060753) and [ROS Q&A](https://answers.ros.org/question/56686/opencv-cmake-error/).

Install OpenCV 3.3.1
```
$ sudo apt-get install libopencv-dev python-opencv
```

Check whether it is installed properly or not by typing
```
$ opencv_version
```

Here is an example CMakeLists.txt and package.xml (for ROS). Just check the OpenCV related parts

CMakeLists.txt:
```
cmake_minimum_required(VERSION 2.8.3)
project(kamu_robotu_map_analyzer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)
find_package(OpenCV REQUIRED)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(map_analyzer src/image_processing.cpp)
target_link_libraries(map_analyzer ${catkin_LIBRARIES} ${OpenCV_LIBS})
```

package.xml:
```
<?xml version="1.0"?>
<package format="2">
  <name>kamu_robotu_map_analyzer</name>
  <version>1.0.0</version>
  <description>The kamu_robotu_map_analyzer package</description> 
  <maintainer email="sami.akgun@ieee.metu.edu.tr">alperen</maintainer>
  <license>MIT</license>
 
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>message_generation</build_depend> 
  <build_depend>sensor_msgs</build_depend>
  <build_depend>image_transport</build_depend>
  <build_depend>cv_bridge</build_depend>
  <build_depend>OpenCV</build_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>image_transport</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>message_runtime</exec_depend>

  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
</package>
```

### Installation Using Source
[Here](https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html) is a reference source.
[Here](https://www.learnopencv.com/install-opencv3-on-ubuntu/) is an another reference source that I followed.

First install dependencies
```
$ sudo apt-get install build-essential
$ sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
$ sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
```

Install OpenCV 3.3.1 
```
$ cd ~/your_desired_location
$ git clone https://github.com/opencv/opencv.git
$ cd opencv 
$ git checkout 3.3.1 
$ cd ..
$ git clone https://github.com/opencv/opencv_contrib.git
$ cd opencv_contrib
$ git checkout 3.3.1
$ cd ..
```

Build using CMake
```
$ cd ~/your_desired_location/opencv
$ mkdir build
$ cd build
$ cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
```

**NOTE:** If above cmake code does not work, try this one:
```
$ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
```

Install libraries (takes very long time!)
```
$ sudo make install
```

You need to change OpenCV_DIR environment variable to specify OpenCV location. Add the following to CMakeLists.txt 
```
$ set( OpenCV_DIR "~/your_desired_location/opencv/build" )
```

Here is an example CMakeLists.txt:
```
cmake_minimum_required(VERSION 2.8.3)
project(kamu_robotu_map_analyzer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set( OpenCV_DIR "~/Programs/Bachelor_Project/opencv/build" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
  OpenCV
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(map_analyzer src/image_processing.cpp)
target_link_libraries(map_analyzer ${catkin_LIBRARIES} ${OpenCV_LIBS})
```




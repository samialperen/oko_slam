cmake_minimum_required(VERSION 2.8.3)
project(kamu_robotu_map_analyzer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


#set( OpenCV_DIR "~/Programs/Bachelor_Project/opencv/build" )

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

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
#  cv_bridge
#  image_transport
  message_generation
#  OpenCV
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


#add_executable(map_analyzer src/image_processing.cpp)
#target_link_libraries(map_analyzer ${catkin_LIBRARIES} ${OpenCV_LIBS})


#add_executable(binary_converter src/binary_converter.cpp src/binary_converter_node.cpp src/ParseArg.cpp)
#target_link_libraries(binary_converter ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(image_processing src/image_processing.cpp src/ParseArg.cpp)
#target_link_libraries(image_processing ${catkin_LIBRARIES} ${OpenCV_LIBS})

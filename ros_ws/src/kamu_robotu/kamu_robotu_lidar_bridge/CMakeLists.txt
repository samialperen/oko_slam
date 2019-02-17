cmake_minimum_required(VERSION 2.8.3)
project(kamu_robotu_lidar_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
	sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(laser_scan_publisher src/laser_scan_publisher.cpp)

target_link_libraries(laser_scan_publisher
  ${catkin_LIBRARIES}
)

# This ensures that sensor_msgs' msg headers are built before your executable
if(sensor_msgs_EXPORTED_TARGETS)
  add_dependencies(laser_scan_publisher ${sensor_msgs_EXPORTED_TARGETS})
endif()
# Link against the catkin_LIBRARIES
target_link_libraries(laser_scan_publisher ${catkin_LIBRARIES})

# Mark executables and/or libraries for installation
install(TARGETS laser_scan_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

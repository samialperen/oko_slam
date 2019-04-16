cmake_minimum_required(VERSION 2.8.3)
project(kamu_robotu_comm)

find_package(catkin REQUIRED COMPONENTS roscpp serial std_msgs sensor_msgs geometry_msgs nav_msgs tf )

include_directories(include ${catkin_INCLUDE_DIRS})

add_service_files(
  FILES
  kamu_cmd.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(LIBRARIES  ${PROJECT_NAME} CATKIN_DEPENDS serial std_msgs sensor_msgs geometry_msgs nav_msgs DEPENDS okoserial)

add_library(okoserial src/okoserial.c)

add_executable(lidar_bridge src/lidar_bridge/lidar_bridge.cpp src/okoserial.c)
target_link_libraries(lidar_bridge ${catkin_LIBRARIES} okoserial)

add_executable(odometry_bridge src/odometry_bridge/odometry_bridge.cpp src/okoserial.c) 
target_link_libraries(odometry_bridge ${catkin_LIBRARIES} okoserial)

# This ensures that sensor_msgs' msg headers are built before your executable
if(sensor_msgs_EXPORTED_TARGETS)
  add_dependencies(lidar_bridge ${sensor_msgs_EXPORTED_TARGETS})
endif()

# Mark executables and/or libraries for installation
install(TARGETS lidar_bridge odometry_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)


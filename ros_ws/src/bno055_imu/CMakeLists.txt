cmake_minimum_required(VERSION 2.8.3)
project(bno055_imu)



## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs std_msgs nav_msgs tf)



catkin_package( LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf DEPENDS getbno055)

include_directories(include ${catkin_INCLUDE_DIRS} )


add_library(getbno055 src/getbno055.c)

add_executable(bno055_imu_node src/bno055_imu_node.cpp src/getbno055.c )

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(mpu6050_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(bno055_imu_node  ${catkin_LIBRARIES} getbno055 )

# This ensures that sensor_msgs' msg headers are built before your executable
if(sensor_msgs_EXPORTED_TARGETS)
  add_dependencies(bno055_imu_node ${sensor_msgs_EXPORTED_TARGETS})
endif()
#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS bno055_imu_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

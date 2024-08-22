cmake_minimum_required(VERSION 3.0.2)
project(ar_path_nav)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  nav_msgs
  ar_track_alvar_msgs
  tf2
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs cv_bridge geometry_msgs std_msgs tf rospy tf2 ar_track_alvar_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(path_generator_node src/path_generator_node.cpp)
add_executable(path_tracker_node src/path_tracker_node.cpp)

target_link_libraries(path_generator_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)

target_link_libraries(path_tracker_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)

#############
## Install ##
#############

install(TARGETS path_generator_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS path_tracker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

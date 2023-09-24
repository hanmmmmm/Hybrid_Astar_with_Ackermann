cmake_minimum_required(VERSION 3.0.2)
project(hawa_ackermann_sim_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf2
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hawa_ackermann_sim_control
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(path_segmenter_test_node 
  src/test/path_segment_test_node.cpp
)

target_link_libraries( path_segmenter_test_node
  ${catkin_LIBRARIES}
)

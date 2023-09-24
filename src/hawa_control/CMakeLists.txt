cmake_minimum_required(VERSION 3.0.2)
project(hawa_control)

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

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(purepurpuit_test_node 
  src/test/node_test_purepersuit.cpp
)

target_link_libraries( purepurpuit_test_node
  ${catkin_LIBRARIES}
)


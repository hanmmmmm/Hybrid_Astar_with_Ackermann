cmake_minimum_required(VERSION 3.0.2)
project(hawa_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf2
)

find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hawa_pathplan
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(purepursuit_node 
  src/node_purepursuit.cpp
)

target_link_libraries( purepursuit_node
  ${catkin_LIBRARIES}
)


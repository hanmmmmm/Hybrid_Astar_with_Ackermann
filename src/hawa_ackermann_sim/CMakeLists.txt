cmake_minimum_required(VERSION 3.0.2)
project(hawa_ackermann_sim)

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
#  LIBRARIES ackermann_sim
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(akm_sim_node 
  src/simulation.cpp
)

target_link_libraries( akm_sim_node
  ${catkin_LIBRARIES}
)
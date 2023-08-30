cmake_minimum_required(VERSION 3.0.2)
project(auxiliary_tools)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES auxiliary_tools
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pc_load_node 
  src/node_pc_load.cpp
)

target_link_libraries( pc_load_node
  ${catkin_LIBRARIES}
)
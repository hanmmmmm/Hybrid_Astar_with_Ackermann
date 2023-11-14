cmake_minimum_required(VERSION 3.0.2)
project(hawa_pathplan)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")  

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf2
  roslib
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
)

add_executable(map_fusion_node 
  src/map_fusion_node.cpp
)

target_link_libraries( map_fusion_node
  ${catkin_LIBRARIES}
)

add_executable(path_plan_node 
  src/path_plan_node.cpp
)

target_link_libraries( path_plan_node
  ${catkin_LIBRARIES}
  # ${cppad_lib} ${colpack_libs} ipopt
)

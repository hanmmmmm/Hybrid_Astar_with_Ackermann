cmake_minimum_required(VERSION 3.0.2)
project(car_basics)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  nav_msgs
  roscpp
  serial
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

include_directories(include/${PROJECT_NAME})

add_executable(wheeltec_robot_node src/wheeltec_robot.cpp src/Quaternion_Solution.cpp)
target_link_libraries(wheeltec_robot_node ${catkin_LIBRARIES})

add_executable(odom_from_tf_node src/odom_from_tf.cpp )
target_link_libraries(odom_from_tf_node ${catkin_LIBRARIES})


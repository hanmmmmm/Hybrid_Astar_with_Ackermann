cmake_minimum_required(VERSION 3.8)
project(hawa_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

include_directories(include)
include_directories("/usr/include/eigen3")


find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


######### Pure Pursuit #########

add_executable(node_purepursuit  src/node_purepursuit.cpp
                                  src/pure_pursuit/class_node_pure_pursuit.cpp
                                  src/pure_pursuit/class_purepursuit.cpp
                                  src/common/class_elemental_pose2d.cpp
                                  src/common/class_multi_segment_manager.cpp
                                  src/common/class_elemental_path2d.cpp
                                  src/common/class_elemental_path2d_segment.cpp
                                  src/common/tools_angles.cpp)

ament_target_dependencies(
  node_purepursuit 
  rclcpp 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  ackermann_msgs
  nav_msgs
  visualization_msgs
  )

target_include_directories(node_purepursuit PRIVATE "/home/jf/Hybrid_Astar_with_Ackermann/src/hawa_control/include/common")

install(TARGETS
        node_purepursuit
        DESTINATION lib/${PROJECT_NAME})


######### LQR #########

add_executable(node_lqr  src/node_lqr.cpp
                         src/lqr/class_node_lqr.cpp
                         src/lqr/class_lqr_ref_point_selector.cpp
                         src/lqr/class_lqr_controller.cpp
                         src/common/class_elemental_pose2d.cpp
                         src/common/class_multi_segment_manager.cpp
                         src/common/class_elemental_path2d.cpp
                         src/common/class_elemental_path2d_segment.cpp
                         src/common/tools_angles.cpp)


ament_target_dependencies(
  node_lqr 
  rclcpp 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  ackermann_msgs
  nav_msgs
  visualization_msgs
  )

target_include_directories(node_lqr PRIVATE "/home/jf/Hybrid_Astar_with_Ackermann/src/hawa_control/include/common")

install(TARGETS
node_lqr
        DESTINATION lib/${PROJECT_NAME})
        

########### Google Test ###########

# find_package(GTest REQUIRED)
# enable_testing()

# include_directories("/opt/ros/humble/include/tf2_ros")
# include_directories("/opt/ros/humble/include/tf2")

# add_executable(
#   test_tools_angles
#   src/test/test_tools_angles.cpp
#   src/common/tools_angles.cpp
# )

# target_link_libraries(
#   test_tools_angles
#   GTest::gtest_main
#   rclcpp
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
# )

# include(GoogleTest)
# gtest_discover_tests(test_tools_angles)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

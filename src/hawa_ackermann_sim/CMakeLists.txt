cmake_minimum_required(VERSION 3.8)
project(hawa_ackermann_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(akm_sim_node  src/simulation.cpp)

ament_target_dependencies(
  akm_sim_node 
  rclcpp 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  ackermann_msgs
  nav_msgs
  visualization_msgs
  )

target_include_directories(akm_sim_node PRIVATE "/home/jf/Hybrid_Astar_with_Ackermann/src/hawa_ackermann_sim/include")

install(TARGETS
        akm_sim_node
        DESTINATION lib/${PROJECT_NAME})

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


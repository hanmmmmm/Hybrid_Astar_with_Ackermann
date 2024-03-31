cmake_minimum_required(VERSION 3.8)
project(hawa_pathplan)

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
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)


add_executable(path_plan_node  src/path_plan_node.cpp)

ament_target_dependencies(
  path_plan_node 
  rclcpp 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  ackermann_msgs
  nav_msgs
  visualization_msgs
  )
ament_target_dependencies(path_plan_node ament_index_cpp)


target_include_directories(path_plan_node PRIVATE "/home/jf/Hybrid_Astar_with_Ackermann/src/hawa_pathplan/include")

install(TARGETS
        path_plan_node
        DESTINATION lib/${PROJECT_NAME})


add_executable(map_fusion_node  src/map_fusion_node.cpp)

ament_target_dependencies(
  map_fusion_node 
  rclcpp 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  ackermann_msgs
  nav_msgs
  visualization_msgs
  tf2_geometry_msgs
  )

target_include_directories(map_fusion_node PRIVATE "/home/jf/ros2_hawa_ws/src/hawa_pathplan/include")

install(TARGETS
        map_fusion_node
        DESTINATION lib/${PROJECT_NAME})

                
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)

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

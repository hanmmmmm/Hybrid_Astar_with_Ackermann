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




include_directories(include )

add_executable(akm_sim_node  src/simulation.cpp
                              src/class_node_ackermann_simulation.cpp
                              src/class_car_body_box_marker.cpp
                              src/class_akm_sim_dynamic_states.cpp
                              src/hawa_sim_tools.cpp
                              src/class_random_sin_noise_generator.cpp
                              )

# target_sources(akm_sim_node PUBLIC src/class_node_ackermann_simulation.cpp)

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

# target_include_directories(akm_sim_node PRIVATE "/home/jf/Hybrid_Astar_with_Ackermann/src/hawa_ackermann_sim/include")

install(TARGETS
        akm_sim_node
        DESTINATION lib/${PROJECT_NAME})


# target_link_libraries(akm_sim_node
# akm_sim_modules
# )



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


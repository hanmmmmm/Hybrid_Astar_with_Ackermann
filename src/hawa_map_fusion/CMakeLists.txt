cmake_minimum_required(VERSION 3.8)
project(hawa_map_fusion)

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
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

include_directories(include)

add_executable(map_fusion_node  
                src/node_map_fusion.cpp     
                src/class_tool_general.cpp 
                src/class_tool_timer.cpp
                src/class_inflation_samples.cpp  
                src/class_map_fusion.cpp    
)

ament_target_dependencies(
  map_fusion_node 
  rclcpp 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  nav_msgs
  visualization_msgs
  tf2_geometry_msgs
  )

install(TARGETS
        map_fusion_node
        DESTINATION lib/${PROJECT_NAME})

                
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)

########### Google Test ###########

find_package(GTest REQUIRED)
enable_testing()

add_executable(
  test_calcOccValue
  src/test/test_calcOccValue.cpp
  src/class_tool_general.cpp
)
target_link_libraries(
  test_calcOccValue
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(test_calcOccValue)



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

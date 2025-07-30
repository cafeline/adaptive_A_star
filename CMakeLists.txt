cmake_minimum_required(VERSION 3.8)
project(adaptive_A_star)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(adaptive_A_star 
  src/adaptive_A_star.cpp
  src/astar_planner.cpp
  src/types.cpp)
target_include_directories(adaptive_A_star PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(adaptive_A_star PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Link ROS 2 dependencies
ament_target_dependencies(adaptive_A_star 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  std_msgs 
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs)

install(TARGETS adaptive_A_star
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

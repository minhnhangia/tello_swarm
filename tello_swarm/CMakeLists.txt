cmake_minimum_required(VERSION 3.8)
project(tello_swarm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(swarm_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)


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

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Add shared base library for swarm servers
add_library(swarm_trigger_base src/swarm_trigger_base.cpp)
ament_target_dependencies(
  swarm_trigger_base
  rclcpp
  std_srvs
)
target_include_directories(swarm_trigger_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Add node takeoff_server
add_executable(takeoff_server src/takeoff_server.cpp)
target_link_libraries(takeoff_server swarm_trigger_base)
ament_target_dependencies(
  takeoff_server
  rclcpp
  std_srvs
)
target_include_directories(takeoff_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Add node landing_server
add_executable(landing_server src/landing_server.cpp)
target_link_libraries(landing_server swarm_trigger_base)
ament_target_dependencies(
  landing_server
  rclcpp
  std_srvs
)
target_include_directories(landing_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(marker_manager src/marker_manager.cpp)
ament_target_dependencies(
  marker_manager
  rclcpp
  std_msgs
  swarm_interfaces
)
target_include_directories(marker_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(waypoint_server src/waypoint_server.cpp)
target_link_libraries(waypoint_server yaml-cpp)
ament_target_dependencies(
  waypoint_server
  rclcpp
  std_msgs
  swarm_interfaces
)
target_include_directories(waypoint_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS 
  takeoff_server
  landing_server
  marker_manager
  waypoint_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()

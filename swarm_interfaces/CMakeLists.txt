cmake_minimum_required(VERSION 3.8)
project(swarm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Add here your custom message (.msg), service (.srv), and action (.action) files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MarkerInfo.msg"
  "msg/MarkerRegistry.msg"
  "srv/ReserveMarker.srv"
  "srv/MarkLanded.srv"
  "srv/QueryTurn.srv"
  "srv/SignalComplete.srv"
  "srv/ReportFailure.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

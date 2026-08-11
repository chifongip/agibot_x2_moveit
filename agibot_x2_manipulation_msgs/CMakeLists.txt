cmake_minimum_required(VERSION 3.22)
project(agibot_x2_manipulation_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Pick.action"
  "action/Place.action"
  "action/PickPlace.action"
  "action/ResetManipulation.action"
  "msg/ManipulationState.msg"
  "srv/RecoverManipulationState.srv"
  DEPENDENCIES geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

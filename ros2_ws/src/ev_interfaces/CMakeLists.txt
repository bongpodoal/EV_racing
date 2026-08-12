cmake_minimum_required(VERSION 3.8)
project(ev_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleInfo.msg"
  "msg/VehicleCommand.msg"
  "msg/Cone.msg"
  "msg/ConeArray.msg"
  DEPENDENCIES std_msgs
)

ament_package()

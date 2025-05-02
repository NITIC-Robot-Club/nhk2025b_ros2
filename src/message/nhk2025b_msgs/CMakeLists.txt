cmake_minimum_required(VERSION 3.8)
project(nhk2025b_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Swerve.msg"
  "msg/RobotStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_auto_package()

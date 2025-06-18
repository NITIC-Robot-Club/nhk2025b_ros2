cmake_minimum_required(VERSION 3.8)
project(nhk2025b_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Swerve.msg"
  "msg/RobotStatus.msg"
  "msg/Command.msg"
  "msg/State.msg"
  "msg/StateArray.msg"
  "msg/BoxInfo.msg"
  "msg/Box.msg"
  "msg/BoxArray.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_auto_package()

cmake_minimum_required(VERSION 3.8)
project(nhk2025b_box_perception)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

target_include_directories(${PROJECT_NAME}
    SYSTEM PUBLIC
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "box_perception::box_perception"
    PLUGIN "box_initializer::box_initializer"
    EXECUTABLE box_perception
)

add_executable(box_initializer src/box_initializer.cpp)
ament_target_dependencies(box_initializer
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nhk2025b_msgs
)

install(TARGETS box_initializer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if($ENV{ROS_DISTRO} STREQUAL "iron" OR $ENV{ROS_DISTRO} STREQUAL "jazzy" OR $ENV{ROS_DISTRO} STREQUAL "rolling")
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
else()
  ament_auto_package(INSTALL_TO_SHARE)
endif()
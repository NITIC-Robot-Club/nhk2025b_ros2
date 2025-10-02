
cmake_minimum_required(VERSION 3.8)
project(nhk2025b_simulation)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

target_include_directories(${PROJECT_NAME}
    SYSTEM PUBLIC
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "simulation::simulation"
    EXECUTABLE simulation
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "lidar_simulation::lidar_simulation"
    EXECUTABLE lidar_simulation
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "box_state_simulation::box_state_simulation"
    EXECUTABLE box_state_simulation  
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "arm_simulation::arm_simulation"
    EXECUTABLE arm_simulation
)

if($ENV{ROS_DISTRO} STREQUAL "iron" OR $ENV{ROS_DISTRO} STREQUAL "jazzy" OR $ENV{ROS_DISTRO} STREQUAL "rolling")
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
else()
  ament_auto_package(INSTALL_TO_SHARE)
endif()

cmake_minimum_required(VERSION 3.8)
project(nhk2025b_ekf)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

target_include_directories(${PROJECT_NAME}
    SYSTEM PUBLIC
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "ekf::ekf"
    EXECUTABLE ekf
)

ament_auto_package(
    INSTALL_TO_SHARE
)
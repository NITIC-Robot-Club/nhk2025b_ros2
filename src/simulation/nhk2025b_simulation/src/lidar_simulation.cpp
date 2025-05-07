#include "nhk2025b_simulation/lidar_simulation.hpp"

namespace lidar_simulation {

lidar_simulation::lidar_simulation (const rclcpp::NodeOptions &options)
    : Node ("lidar_simulation", options), tf_buffer (this->get_clock ()), tf_listener (tf_buffer) {
    pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/localization/pose", 10, std::bind (&lidar_simulation::pose_callback, this, std::placeholders::_1));
    map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid> (
        "/map", 10, std::bind (&lidar_simulation::map_callback, this, std::placeholders::_1));
    laser_publisher = this->create_publisher<sensor_msgs::msg::LaserScan> ("/scan", 10);
    timer           = this->create_wall_timer (std::chrono::milliseconds (100), std::bind (&lidar_simulation::timer_callback, this));
}
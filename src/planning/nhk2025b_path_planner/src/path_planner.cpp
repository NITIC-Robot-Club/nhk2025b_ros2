#include "nhk2025b_path_planner/path_planner.hpp"

namespace path_planner {
path_planner::path_planner (const rclcpp::NodeOptions &options) : Node ("path_planner", options) {
    resolution_ms             = this->declare_parameter<int> ("resolution_ms", 10);
    offset_mm                 = this->declare_parameter<int> ("offset_mm", 50);
    robot_size_mm             = this->declare_parameter<int> ("robot_size_mm", 1414);
    max_xy_acceleration_m_s2  = this->declare_parameter<double> ("max_xy_acceleration_m_s2", 10.0);
    max_xy_velocity_m_s       = this->declare_parameter<double> ("max_xy_velocity_m_s", 1.5);
    max_z_acceleration_rad_s2 = this->declare_parameter<double> ("max_z_acceleration_rad_s2", 10.0);
    max_z_velocity_rad_s      = this->declare_parameter<double> ("max_z_velocity_rad_s", 3.14);

    path_publisher          = this->create_publisher<nav_msgs::msg::Path> ("/planning/path", 10);
    current_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/localization/current_pose", 10, std::bind (&path_planner::current_pose_callback, this, std::placeholders::_1));
    goal_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/bt/goal_pose", 10, std::bind (&path_planner::goal_pose_callback, this, std::placeholders::_1));
    map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid> (
        "/bt/map", 10, std::bind (&path_planner::map_callback, this, std::placeholders::_1));
    timer = this->create_wall_timer (std::chrono::milliseconds (resolution_ms), std::bind (&path_planner::timer_callback, this));
}
void path_planner::timer_callback () {}

void path_planner::current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose.header = msg->header;
    current_pose.pose = msg->pose;
}

void path_planner::goal_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_pose.header = msg->header;
    goal_pose.pose = msg->pose;
}

void path_planner::map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map.header = msg->header;
    map.info = msg->info;
}
}  // namespace path_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (path_planner::path_planner)
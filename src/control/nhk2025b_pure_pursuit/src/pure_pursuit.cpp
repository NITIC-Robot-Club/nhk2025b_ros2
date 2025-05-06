#include "nhk2025b_pure_pursuit/pure_pursuit.hpp"

namespace pure_pursuit {
pure_pursuit::pure_pursuit (const rclcpp::NodeOptions &options) : Node ("pure_pursuit", options) {
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped> ("/cmd_vel", 10);
    pose_subscriber_   = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/current_pose", 10, std::bind (&pure_pursuit::pose_callback, this, std::placeholders::_1));
    path_subscriber_ =
        this->create_subscription<nav_msgs::msg::Path> ("/path", 10, std::bind (&pure_pursuit::path_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer (std::chrono::milliseconds (50), std::bind (&pure_pursuit::timer_callback, this));

    this->declare_parameter ("lookahead_distance", 0.5);
    this->declare_parameter ("angle_p", 1.0);
}

void pure_pursuit::pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
}

void pure_pursuit::path_callback (const nav_msgs::msg::Path::SharedPtr msg) {
    path_ = *msg;
}

void pure_pursuit::timer_callback () {
    this->get_parameter ("lookahead_distance", lookahead_distance_);
    this->get_parameter ("angle_p", angle_p_);
}

}  // namespace pure_pursuit

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (pure_pursuit::pure_pursuit)
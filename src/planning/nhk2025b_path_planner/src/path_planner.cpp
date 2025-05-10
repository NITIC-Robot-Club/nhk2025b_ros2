#include "nhk2025b_path_planner/path_planner.hpp"

namespace path_planner {
path_planner::path_planner (const rclcpp::NodeOptions &options) : Node ("path_planner", options) {
    resolution_ms             = this->declare_parameter<int> ("resolution_ms", 10);
    offset_mm                 = this->declare_parameter<int> ("offset_mm", 50);
    robot_size_mm             = this->declare_parameter<int> ("robot_size_mm", 1414);
    max_xy_acceleration_m_s2  = this->declare_parameter<double> ("max_xy_acceleration_m_s2", 2.0);
    max_xy_velocity_m_s       = this->declare_parameter<double> ("max_xy_velocity_m_s", 1.5);
    max_z_acceleration_rad_s2 = this->declare_parameter<double> ("max_z_acceleration_rad_s2", 10.0);
    max_z_velocity_rad_s      = this->declare_parameter<double> ("max_z_velocity_rad_s", 3.14);

    path_publisher          = this->create_publisher<nav_msgs::msg::Path> ("/planning/path", 10);
    current_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/simulation/pose", 10, std::bind (&path_planner::current_pose_callback, this, std::placeholders::_1));
    goal_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/behavior/goal_pose", 10, std::bind (&path_planner::goal_pose_callback, this, std::placeholders::_1));
    map_subscriber =
        this->create_subscription<nav_msgs::msg::OccupancyGrid> ("/bt/map", 10, std::bind (&path_planner::map_callback, this, std::placeholders::_1));
    vel_subscriber = this->create_subscription<geometry_msgs::msg::TwistStamped> (
        "/cmd_vel", 10, std::bind (&path_planner::vel_callback, this, std::placeholders::_1));
    timer = this->create_wall_timer (std::chrono::milliseconds (100), std::bind (&path_planner::timer_callback, this));
}
void path_planner::timer_callback () {
    nav_msgs::msg::Path   path;
    std_msgs::msg::Header header;
    header.frame_id = "map";
    header.stamp    = this->now ();
    // a = 1m/s2 , v = 2m/s ,x=6m
    double err_x     = goal_pose.pose.position.x - current_pose.pose.position.x;
    double err_y     = goal_pose.pose.position.y - current_pose.pose.position.y;
    double distance  = std::hypot (err_x, err_y);
    double err_angle = std::atan2 (err_y, err_x);

    double current_yaw = 2.0 * std::asin (current_pose.pose.orientation.z);
    double goal_yaw    = 2.0 * std::asin (goal_pose.pose.orientation.z);
    double delta_yaw   = goal_yaw - current_yaw;
    if (delta_yaw > M_PI)
        delta_yaw -= 2 * M_PI;
    else if (delta_yaw < -M_PI)
        delta_yaw += 2 * M_PI;

    double x = 0, y = 0;
    double v_x = 1.0, v_y = 0;
    path.header    = header;
    double delta_t = resolution_ms / 1000.0;
    bool   decel_x = false, decel_y = false;
    double limit_acceleration = 0.9 * max_xy_acceleration_m_s2;
    double v_prev             = 0.0;
    for (double t = 0; x <= err_x && y <= err_y; t += delta_t) {
        x += v_x * delta_t;
        y += v_y * delta_t;
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = current_pose.pose.position.x + x;
        // pose.pose.position.x = t;
        pose.pose.position.y = current_pose.pose.position.y + y;
        // pose.pose.position.y = v;
        double now_yaw          = current_yaw + delta_yaw / (1.0 + std::exp (-7.5 * (hypot (x, y) / distance - 0.5)));
        pose.pose.orientation.z = std::sin (now_yaw / 2.0);
        pose.pose.orientation.w = std::cos (now_yaw / 2.0);

        rclcpp::Time time (header.stamp);
        rclcpp::Time new_time = time + rclcpp::Duration::from_seconds (delta_t);

        header.stamp = new_time;
        pose.header  = header;
        path.poses.push_back (pose);
        if (decel_x) {
            v_x = std::sqrt (2 * limit_acceleration * (err_x - x));
        } else if (err_x - x <= v_x * v_x / (2 * limit_acceleration)) {
            decel_x = true;
            v_x     = std::sqrt (2 * limit_acceleration * (err_x - x));
        } else if (v_x < max_xy_velocity_m_s) {
            v_x += max_xy_acceleration_m_s2 * delta_t;
        } else {
            v_x = max_xy_velocity_m_s;
        }

        if (decel_y) {
            v_y = std::sqrt (2 * limit_acceleration * (err_y - y));
        } else if (err_y - y <= v_y * v_y / (2 * limit_acceleration)) {
            decel_y = true;
            v_y     = std::sqrt (2 * limit_acceleration * (err_y - y));
        } else if (v_y < max_xy_velocity_m_s) {
            v_y += max_xy_acceleration_m_s2 * delta_t;
        } else {
            v_y = max_xy_velocity_m_s;
        }
        // v = std::abs(v);
    }
    path_publisher->publish (path);
}

void path_planner::current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose = *msg;
}

void path_planner::goal_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_pose = *msg;
}

void path_planner::map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map = *msg;
}

void path_planner::vel_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    current_vel = *msg;
}
}  // namespace path_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (path_planner::path_planner)
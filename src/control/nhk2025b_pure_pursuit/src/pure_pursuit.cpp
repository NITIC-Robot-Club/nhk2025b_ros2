#include "nhk2025b_pure_pursuit/pure_pursuit.hpp"

namespace pure_pursuit {

pure_pursuit::pure_pursuit (const rclcpp::NodeOptions &options) : Node ("pure_pursuit", options) {
    cmd_vel_publisher_    = this->create_publisher<geometry_msgs::msg::TwistStamped> ("/control/cmd_vel", 1);
    look_ahead_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/control/look_ahead_pose", 1);
    pose_subscriber_      = this->create_subscription<geometry_msgs::msg::PoseStamped> ("/localization/current_pose", 1, std::bind (&pure_pursuit::pose_callback, this, std::placeholders::_1));
    path_subscriber_      = this->create_subscription<nav_msgs::msg::Path> ("/planning/path", 1, std::bind (&pure_pursuit::path_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer (std::chrono::milliseconds (delta_t_ms), std::bind (&pure_pursuit::timer_callback, this));

    this->declare_parameter ("lookahead_time", 1.0);
    this->declare_parameter ("min_look_ahead_distance", 0.3);
    this->declare_parameter ("max_look_ahead_distance", 5.0);
    this->declare_parameter ("max_speed_xy_m_s", 4.0);
    this->declare_parameter ("max_speed_z_rad_s", 3.14);
    this->declare_parameter ("max_acceleration_xy_m_s2_", 10.0);
    this->declare_parameter ("max_acceleration_z_rad_s2", 6.28);
    this->declare_parameter ("goal_deceleration_xy_m_s2", 3.0);
}

void pure_pursuit::pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
}

void pure_pursuit::path_callback (const nav_msgs::msg::Path::SharedPtr msg) {
    path_ = *msg;
}

void pure_pursuit::timer_callback () {
    // パラメータ取得
    this->get_parameter ("lookahead_time", lookahead_time_);
    this->get_parameter ("min_look_ahead_distance", min_look_ahead_distance_);
    this->get_parameter ("max_look_ahead_distance", max_look_ahead_distance_);
    this->get_parameter ("max_speed_xy_m_s", max_speed_xy_m_s_);
    this->get_parameter ("max_speed_z_rad_s", max_speed_z_rad_s_);
    this->get_parameter ("max_acceleration_xy_m_s2_", max_acceleration_xy_m_s2_);
    this->get_parameter ("max_acceleration_z_rad_s2", max_acceleration_z_rad_s2_);
    this->get_parameter ("goal_deceleration_xy_m_s2", goal_deceleration_xy_m_s2_);

    // からの経路は停止
    if (path_.poses.empty ()) {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp    = this->now ();
        cmd_vel.header.frame_id = "base_link";
        cmd_vel.twist.linear.x  = 0.0;
        cmd_vel.twist.linear.y  = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        cmd_vel_publisher_->publish (cmd_vel);
        return;
    }

    int    current_index       = calculate_index (current_pose_.pose);
    double look_ahead_distance = calculate_look_ahead_distance ();

    geometry_msgs::msg::Pose look_ahead_pose = calculate_look_ahead_pose (look_ahead_distance);

    double path_distance = calculate_path_distance (current_index);
    double max_speed     = calculate_max_speed (path_distance);

    geometry_msgs::msg::TwistStamped twist;
    twist.twist           = calculate_xy (look_ahead_pose, max_speed);
    twist.twist.angular.z = calculate_z (nhk2025b_utils::get_yaw_2d (look_ahead_pose.orientation));
    twist.header.frame_id = "base_link";
    twist.header.stamp = this->now();
    cmd_vel_publisher_->publish(twist);

    geometry_msgs::msg::PoseStamped look_ahead;
    look_ahead.header.frame_id = "map";
    look_ahead.header.stamp = this->now();
    look_ahead.pose = look_ahead_pose;
    look_ahead_publisher_->publish(look_ahead);

    last_cmd_vel_ = twist;
}

double pure_pursuit::calculate_speed (geometry_msgs::msg::Twist twist) {
    return hypot (twist.linear.x, twist.linear.y);
}

int pure_pursuit::calculate_index (geometry_msgs::msg::Pose pose) {
    double min_diff       = 0.0;
    int    min_diff_index = -1;
    for (int index = 0; index < path_.poses.size (); index++) {
        geometry_msgs::msg::Point position = path_.poses[index].pose.position;

        double diff = hypot (position.x - pose.position.x, position.y - pose.position.y);
        if (diff < min_diff || min_diff_index == -1) {
            min_diff       = diff;
            min_diff_index = index;
        }
    }
    return min_diff_index;
}

double pure_pursuit::calculate_look_ahead_distance () {
    double look_ahead_distance = lookahead_time_ * calculate_speed (last_cmd_vel_.twist);

    look_ahead_distance = std::max (std::min (look_ahead_distance, max_look_ahead_distance_), min_look_ahead_distance_);
    return look_ahead_distance;
}

int pure_pursuit::calculate_look_ahead_index (double look_ahead_distance) {
    double min_diff       = 0.0;
    int    min_diff_index = -1;
    for (int index = 0; index < path_.poses.size (); index++) {
        geometry_msgs::msg::Point position = path_.poses[index].pose.position;

        double diff = abs (hypot (position.x - current_pose_.pose.position.x, position.y - current_pose_.pose.position.y) - look_ahead_distance);
        if (diff < min_diff || min_diff_index == -1) {
            min_diff       = diff;
            min_diff_index = index;
        }
    }
    return min_diff_index;
}

geometry_msgs::msg::Pose pure_pursuit::calculate_look_ahead_pose (int look_ahead_index) {
    return path_.poses[look_ahead_index].pose;
}

double pure_pursuit::calculate_path_distance (int current_index) {
    double path_distance = 0.0;
    for (int index = current_index; index < path_.poses.size () - 1; index++) {
        geometry_msgs::msg::Point index_0 = path_.poses[index].pose.position;
        geometry_msgs::msg::Point index_1 = path_.poses[index + 1].pose.position;
        path_distance += abs (hypot (index_0.x - index_1.x, index_0.y - index_1.y));
    }
    return path_distance;
}

double pure_pursuit::calculate_max_speed (double path_distance) {
    return std::sqrt (2 * goal_deceleration_xy_m_s2_ * path_distance);
}

geometry_msgs::msg::Twist pure_pursuit::calculate_xy (geometry_msgs::msg::Pose look_ahead_pose, double max_speed) {
    double delta_x = look_ahead_pose.position.x - current_pose_.pose.position.x;
    double delta_y = look_ahead_pose.position.y - current_pose_.pose.position.y;

    double current_yaw = nhk2025b_utils::get_yaw_2d (current_pose_.pose.orientation);
    double angle_diff  = std::atan2 (delta_y, delta_x) - current_yaw;

    double speed = std::hypot (delta_x, delta_y) / delta_t_ms * 1000;
    speed        = std::min (std::max (speed, 0.0), max_speed);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = speed * cos (angle_diff);
    twist.linear.y = speed * sin (angle_diff);

    return twist;
}

double pure_pursuit::calculate_z (double look_ahead_z) {
    double delta_z = look_ahead_z - nhk2025b_utils::get_yaw_2d (current_pose_.pose.orientation);
    return delta_z / delta_t_ms * 1000;
}

}  // namespace pure_pursuit

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (pure_pursuit::pure_pursuit)
#include "nhk2025b_pure_pursuit/pure_pursuit.hpp"

namespace pure_pursuit {

pure_pursuit::pure_pursuit (const rclcpp::NodeOptions &options) : Node ("pure_pursuit", options) {
    cmd_vel_publisher_   = this->create_publisher<geometry_msgs::msg::TwistStamped> ("/cmd_vel", 10);
    lookahead_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/control/lookahead_pose", 10);
    pose_subscriber_     = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/localization/current_pose", 10, std::bind (&pure_pursuit::pose_callback, this, std::placeholders::_1));
    path_subscriber_ =
        this->create_subscription<nav_msgs::msg::Path> ("/planning/path", 10, std::bind (&pure_pursuit::path_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer (std::chrono::milliseconds (50), std::bind (&pure_pursuit::timer_callback, this));

    this->declare_parameter ("lookahead_time", 1.0);
    this->declare_parameter ("min_lookahead_distance", 0.3);
    this->declare_parameter ("max_lookahead_distance", 3.0);
    this->declare_parameter ("angle_p", 2.0);
    this->declare_parameter ("max_speed_xy_m_s", 3.0);
    this->declare_parameter ("max_speed_z_rad_s", 3.14);
    this->declare_parameter ("max_acceleration_xy_m_s2_", 6.0);
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
    this->get_parameter ("min_lookahead_distance", min_lookahead_distance_);
    this->get_parameter ("max_lookahead_distance", max_lookahead_distance_);
    this->get_parameter ("angle_p", angle_p_);
    this->get_parameter ("max_speed_xy_m_s", max_speed_xy_m_s_);
    this->get_parameter ("max_speed_z_rad_s", max_speed_z_rad_s_);
    this->get_parameter ("max_acceleration_xy_m_s2_", max_acceleration_xy_m_s2_);

    if (path_.poses.empty ()) {
        RCLCPP_WARN (this->get_logger (), "Path is empty");
        return;
    }

    // 最近傍点の探索
    double min_distance  = std::numeric_limits<double>::max ();
    int    closest_index = -1;
    for (int i = 0; i < path_.poses.size (); i++) {
        double dist = std::hypot (
            path_.poses[i].pose.position.x - current_pose_.pose.position.x, path_.poses[i].pose.position.y - current_pose_.pose.position.y);
        if (dist < min_distance) {
            min_distance  = dist;
            closest_index = i;
        }
    }

    if (closest_index == -1) return;
    if (closest_index + 1 >= path_.poses.size ()) closest_index = path_.poses.size () - 2;

    // ゴール位置
    geometry_msgs::msg::Pose goal_pose = path_.poses.back ().pose;
    double goal_distance = std::hypot (goal_pose.position.x - current_pose_.pose.position.x, goal_pose.position.y - current_pose_.pose.position.y);

    // lookahead点の探索
    int lookahead_index = closest_index;
    for (int i = closest_index; i < path_.poses.size (); i++) {
        double dist = std::hypot (
            path_.poses[i].pose.position.x - current_pose_.pose.position.x, path_.poses[i].pose.position.y - current_pose_.pose.position.y);
        if (dist > lookahead_distance_) break;
        lookahead_index = i;
    }

    if (lookahead_index == closest_index) lookahead_index = path_.poses.size () - 1;

    double dx = path_.poses[lookahead_index].pose.position.x - current_pose_.pose.position.x;
    double dy = path_.poses[lookahead_index].pose.position.y - current_pose_.pose.position.y;

    double current_yaw = std::asin (current_pose_.pose.orientation.z) * 2;
    double angle_diff  = std::atan2 (dy, dx) - current_yaw;
    double yaw_diff    = std::asin (path_.poses[lookahead_index].pose.orientation.z) * 2 - current_yaw;
    if (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
    if (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;

    // 加速度制限付き速度推定
    double delta_t      = 0.05;  // 50ms
    double last_speed   = std::hypot (last_cmd_vel_.twist.linear.x, last_cmd_vel_.twist.linear.y);
    double target_speed = std::hypot (dx, dy) / delta_t;
    double acceleration = (target_speed - last_speed) / delta_t;
    acceleration        = std::clamp (acceleration, -max_acceleration_xy_m_s2_, max_acceleration_xy_m_s2_);
    double speed        = last_speed + acceleration * delta_t;

    // 曲率ベースの速度制限
    double curvature               = 2.0 * std::sin (angle_diff) / std::max (lookahead_distance_, 0.01);
    double curvature_limited_speed = std::sqrt (max_acceleration_xy_m_s2_ / std::max (std::abs (curvature), 1e-3));
    speed                          = std::min (speed, curvature_limited_speed);

    // ゴール付近での減速
    constexpr double goal_tolerance_distance = 0.5;  // [m]
    if (goal_distance < goal_tolerance_distance) {
        speed *= goal_distance / goal_tolerance_distance;
        if (goal_distance < 0.05) speed = 0.0;
    }

    // 最大速度制限
    speed = std::clamp (speed, 0.0, max_speed_xy_m_s_);

    // lookahead距離更新
    lookahead_distance_ = std::clamp (lookahead_time_ * speed, min_lookahead_distance_, max_lookahead_distance_);

    double yaw_speed = angle_p_ * yaw_diff;

    // Twist 発行
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp    = this->now ();
    cmd_vel.header.frame_id = "base_link";
    cmd_vel.twist.linear.x  = speed * std::cos (angle_diff);
    cmd_vel.twist.linear.y  = speed * std::sin (angle_diff);
    cmd_vel.twist.angular.z = yaw_speed;
    cmd_vel_publisher_->publish (cmd_vel);

    // lookahead可視化
    geometry_msgs::msg::PoseStamped lookahead_point;
    lookahead_point.header.stamp    = this->now ();
    lookahead_point.header.frame_id = "map";
    lookahead_point.pose            = path_.poses[lookahead_index].pose;
    lookahead_publisher_->publish (lookahead_point);

    last_cmd_vel_ = cmd_vel;
}

}  // namespace pure_pursuit

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (pure_pursuit::pure_pursuit)
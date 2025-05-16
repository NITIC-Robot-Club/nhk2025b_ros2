#include "nhk2025b_pure_pursuit/pure_pursuit.hpp"

namespace pure_pursuit {

pure_pursuit::pure_pursuit (const rclcpp::NodeOptions &options) : Node ("pure_pursuit", options) {
    cmd_vel_publisher_    = this->create_publisher<geometry_msgs::msg::TwistStamped> ("/cmd_vel", 10);
    lookahead_publisher_  = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/control/lookahead_pose", 10);
    lookahead2_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/control/lookahead_pose2", 10);
    pose_subscriber_      = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/localization/current_pose", 10, std::bind (&pure_pursuit::pose_callback, this, std::placeholders::_1));
    path_subscriber_ =
        this->create_subscription<nav_msgs::msg::Path> ("/planning/path", 10, std::bind (&pure_pursuit::path_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer (std::chrono::milliseconds (50), std::bind (&pure_pursuit::timer_callback, this));

    this->declare_parameter ("lookahead_time", 1.0);
    this->declare_parameter ("min_lookahead_distance", 0.3);
    this->declare_parameter ("max_lookahead_distance", 3.0);
    this->declare_parameter ("angle_lookahead_distance", 1.0);
    this->declare_parameter ("curvature_decceleration_p", 10.0);
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
    this->get_parameter ("angle_lookahead_distance", angle_lookahead_distance_);
    this->get_parameter ("curvature_decceleration_p", curvature_decceleration_p_);
    this->get_parameter ("angle_p", angle_p_);
    this->get_parameter ("max_speed_xy_m_s", max_speed_xy_m_s_);
    this->get_parameter ("max_speed_z_rad_s", max_speed_z_rad_s_);
    this->get_parameter ("max_acceleration_xy_m_s2_", max_acceleration_xy_m_s2_);

    if (path_.poses.empty ()) {
        // RCLCPP_WARN (this->get_logger (), "Path is empty");
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
        if (dist > lookahead_distance_) {
            break;
        }
        lookahead_index = i;
    }

    if (lookahead_index == closest_index) lookahead_index = path_.poses.size () - 1;

    double dx = path_.poses[lookahead_index].pose.position.x - current_pose_.pose.position.x;
    double dy = path_.poses[lookahead_index].pose.position.y - current_pose_.pose.position.y;

    double current_yaw = get_yaw_2d (current_pose_.pose.orientation);
    double angle_diff  = std::atan2 (dy, dx) - current_yaw;

    // 加速度制限付き速度推定
    double delta_t      = 0.05;  // 50ms
    double last_speed   = std::hypot (last_cmd_vel_.twist.linear.x, last_cmd_vel_.twist.linear.y);
    double target_speed = std::hypot (dx, dy) / delta_t;

    // ゴール付近での減速
    constexpr double goal_tolerance_distance = 1;  // [m]
    if (goal_distance < goal_tolerance_distance) {
        target_speed *= goal_distance / goal_tolerance_distance;
        if (goal_distance < 0.05) target_speed = 0.0;
    }

    // 曲率に応じた速度制限
    int p1 = closest_index;
    int p2 = (lookahead_index + closest_index) / 2;
    int p3 = lookahead_index;

    double a = std::hypot (
        path_.poses[p1].pose.position.x - path_.poses[p2].pose.position.x, path_.poses[p1].pose.position.y - path_.poses[p2].pose.position.y);
    double b = std::hypot (
        path_.poses[p2].pose.position.x - path_.poses[p3].pose.position.x, path_.poses[p2].pose.position.y - path_.poses[p3].pose.position.y);
    double c = std::hypot (
        path_.poses[p1].pose.position.x - path_.poses[p3].pose.position.x, path_.poses[p1].pose.position.y - path_.poses[p3].pose.position.y);
    double s         = (a + b + c) / 2.0;
    double area      = std::sqrt (s * (s - a) * (s - b) * (s - c));
    double curvature = 0.0;
    if (a * b * c > 1e-6) {
        curvature = 4.0 * area / (a * b * c);
    }

    double curvature_speed = target_speed / (std::abs (curvature * curvature_decceleration_p_) + 1e-6);
    // RCLCPP_INFO(this->get_logger (), "target speed: %f, curvature speed: %f", target_speed, curvature_speed);
    target_speed = std::min (target_speed, curvature_speed);

    // 最大速度制限
    target_speed = std::clamp (target_speed, 0.0, max_speed_xy_m_s_);

    // lookahead距離更新
    lookahead_distance_ = std::clamp (lookahead_time_ * target_speed, min_lookahead_distance_, max_lookahead_distance_);

    double acceleration = (target_speed - last_speed) / delta_t;
    acceleration        = std::clamp (acceleration, -max_acceleration_xy_m_s2_, max_acceleration_xy_m_s2_);
    double speed        = last_speed + acceleration * delta_t;

    int angle_lookahead_index = closest_index;
    for (int i = closest_index; i < path_.poses.size (); i++) {
        double dist = std::hypot (
            path_.poses[i].pose.position.x - current_pose_.pose.position.x, path_.poses[i].pose.position.y - current_pose_.pose.position.y);
        if (dist > angle_lookahead_distance_) {
            break;
        }
        angle_lookahead_index = i;
    }
    if (angle_lookahead_index == closest_index) angle_lookahead_index = path_.poses.size () - 1;
    double yaw_diff = get_yaw_2d (path_.poses[angle_lookahead_index].pose.orientation) - current_yaw;
    while (yaw_diff > +M_PI) yaw_diff -= 2.0 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
    double yaw_speed = angle_p_ * yaw_diff;
    yaw_speed        = std::clamp (yaw_speed, -max_speed_z_rad_s_, max_speed_z_rad_s_);

    // Twist 発行
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp    = this->now ();
    cmd_vel.header.frame_id = "base_link";
    cmd_vel.twist.linear.x  = speed * std::cos (angle_diff - yaw_speed * delta_t * 3);
    cmd_vel.twist.linear.y  = speed * std::sin (angle_diff - yaw_speed * delta_t * 3);
    cmd_vel.twist.angular.z = yaw_speed;
    cmd_vel_publisher_->publish (cmd_vel);

    // lookahead可視化
    geometry_msgs::msg::PoseStamped lookahead_msg;
    lookahead_msg.header.stamp    = this->now ();
    lookahead_msg.header.frame_id = "map";
    lookahead_msg.pose            = path_.poses[lookahead_index].pose;
    lookahead_publisher_->publish (lookahead_msg);

    geometry_msgs::msg::PoseStamped lookahead2_msg;
    lookahead2_msg.header.stamp    = this->now ();
    lookahead2_msg.header.frame_id = "map";
    lookahead2_msg.pose            = path_.poses[p2].pose;
    lookahead2_publisher_->publish (lookahead2_msg);

    last_cmd_vel_ = cmd_vel;
}

}  // namespace pure_pursuit

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (pure_pursuit::pure_pursuit)
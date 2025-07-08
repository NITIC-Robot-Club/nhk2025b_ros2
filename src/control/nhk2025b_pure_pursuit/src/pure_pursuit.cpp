#include "nhk2025b_pure_pursuit/pure_pursuit.hpp"

namespace pure_pursuit {

pure_pursuit::pure_pursuit (const rclcpp::NodeOptions &options) : Node ("pure_pursuit", options) {
    cmd_vel_publisher_            = this->create_publisher<geometry_msgs::msg::TwistStamped> ("/cmd_vel", 1);
    lookahead_position_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/control/lookahead_position", 1);
    lookahead_angle_publisher_    = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/control/lookahead_angle", 1);
    pose_subscriber_              = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/localization/current_pose", 1, std::bind (&pure_pursuit::pose_callback, this, std::placeholders::_1));
    path_subscriber_ =
        this->create_subscription<nav_msgs::msg::Path> ("/planning/path", 1, std::bind (&pure_pursuit::path_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer (std::chrono::milliseconds (50), std::bind (&pure_pursuit::timer_callback, this));

    this->declare_parameter ("lookahead_time", 1.0);
    this->declare_parameter ("min_lookahead_distance", 0.3);
    this->declare_parameter ("max_lookahead_distance", 5.0);
    this->declare_parameter ("angle_lookahead_distance", 1.0);
    this->declare_parameter ("curvature_decceleration_p", 10.0);
    this->declare_parameter ("angle_p", 2.0);
    this->declare_parameter ("max_speed_xy_m_s", 3.0);
    this->declare_parameter ("max_speed_z_rad_s", 3.14);
    this->declare_parameter ("max_acceleration_xy_m_s2_", 6.0);
    this->declare_parameter ("max_acceleration_z_rad_s2", 6.0);
    this->declare_parameter ("goal_deceleration_distance_p", 1.0);
    this->declare_parameter ("goal_deceleration_p", 2.0);
    this->declare_parameter ("angle_deceleration_p", 2.0);
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
    this->get_parameter ("max_acceleration_z_rad_s2", max_acceleration_z_rad_s2_);
    this->get_parameter ("goal_deceleration_distance_p", goal_deceleration_distance_p_);
    this->get_parameter ("goal_deceleration_p", goal_deceleration_p_);
    this->get_parameter ("angle_deceleration_p", angle_deceleration_p_);

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
    int    lookahead_index     = closest_index;
    double min_lookahead_error = 1e9;
    for (int i = closest_index; i < path_.poses.size (); i++) {
        double dist = std::hypot (
            path_.poses[i].pose.position.x - current_pose_.pose.position.x, path_.poses[i].pose.position.y - current_pose_.pose.position.y);
        double error = std::abs (dist - lookahead_distance_);
        if (error < min_lookahead_error) {
            min_lookahead_error = error;
            lookahead_index     = i;
        }
    }

    double dx = path_.poses[lookahead_index].pose.position.x - current_pose_.pose.position.x;
    double dy = path_.poses[lookahead_index].pose.position.y - current_pose_.pose.position.y;

    double current_yaw = nhk2025b_utils::get_yaw_2d (current_pose_.pose.orientation);
    double angle_diff  = std::atan2 (dy, dx) - current_yaw;

    // 加速度制限付き速度推定
    double delta_t      = 0.05;  // 50ms
    double last_speed   = std::hypot (last_cmd_vel_.twist.linear.x, last_cmd_vel_.twist.linear.y);
    double target_speed = std::hypot (dx, dy) / delta_t;
    // 最大速度制限
    target_speed = std::clamp (target_speed, 0.0, max_speed_xy_m_s_);

    double d = std::max (goal_distance, 0.0);

    // 今の距離dで静止できる最大速度
    double max_stop_speed = std::sqrt (2.0 * max_acceleration_xy_m_s2_ * d / goal_deceleration_distance_p_);

    target_speed = std::min (target_speed, max_stop_speed);

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

    target_speed = std::min (target_speed, curvature_speed);

    double acceleration = (target_speed - last_speed) / delta_t;
    acceleration        = std::clamp (acceleration, -max_acceleration_xy_m_s2_, max_acceleration_xy_m_s2_);
    double speed        = last_speed + acceleration * delta_t;
    lookahead_distance_ = std::clamp (lookahead_time_ * speed, min_lookahead_distance_, max_lookahead_distance_);

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
    double yaw_diff = nhk2025b_utils::get_yaw_2d (path_.poses[angle_lookahead_index].pose.orientation) - current_yaw;
    while (yaw_diff > +M_PI) yaw_diff -= 2.0 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
    double yaw_speed = angle_p_ * yaw_diff;
    // 加速度を考慮
    double angle_acceleration = (yaw_speed - last_cmd_vel_.twist.angular.z) / delta_t;
    angle_acceleration        = std::clamp (angle_acceleration, -max_acceleration_z_rad_s2_, max_acceleration_z_rad_s2_);
    yaw_speed                 = last_cmd_vel_.twist.angular.z + angle_acceleration * delta_t;
    yaw_speed                 = std::clamp (yaw_speed, -max_speed_z_rad_s_, max_speed_z_rad_s_);

    // Twist 発行
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp    = this->now ();
    cmd_vel.header.frame_id = "base_link";
    cmd_vel.twist.linear.x  = speed * std::cos (angle_diff - yaw_speed * delta_t * 3);
    cmd_vel.twist.linear.y  = speed * std::sin (angle_diff - yaw_speed * delta_t * 3);
    cmd_vel.twist.angular.z = yaw_speed;
    cmd_vel_publisher_->publish (cmd_vel);

    // lookahead可視化
    geometry_msgs::msg::PoseStamped lookahead_position_msg;
    lookahead_position_msg.header.stamp    = this->now ();
    lookahead_position_msg.header.frame_id = "map";
    lookahead_position_msg.pose            = path_.poses[lookahead_index].pose;
    lookahead_position_publisher_->publish (lookahead_position_msg);

    geometry_msgs::msg::PoseStamped lookahead_angle_msg;
    lookahead_angle_msg.header.stamp    = this->now ();
    lookahead_angle_msg.header.frame_id = "map";
    lookahead_angle_msg.pose            = path_.poses[angle_lookahead_index].pose;
    lookahead_angle_publisher_->publish (lookahead_angle_msg);

    last_cmd_vel_ = cmd_vel;
}
}  // namespace pure_pursuit

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (pure_pursuit::pure_pursuit)
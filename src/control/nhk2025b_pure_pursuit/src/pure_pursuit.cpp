#include "nhk2025b_pure_pursuit/pure_pursuit.hpp"

namespace pure_pursuit {
pure_pursuit::pure_pursuit (const rclcpp::NodeOptions &options) : Node ("pure_pursuit", options) {
    cmd_vel_publisher_   = this->create_publisher<geometry_msgs::msg::TwistStamped> ("/cmd_vel", 10);
    lookahead_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/control/lookahead_pose", 10);
    pose_subscriber_     = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/simulation/pose", 10, std::bind (&pure_pursuit::pose_callback, this, std::placeholders::_1));

    path_subscriber_ =
        this->create_subscription<nav_msgs::msg::Path> ("/planning/path", 10, std::bind (&pure_pursuit::path_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer (std::chrono::milliseconds (50), std::bind (&pure_pursuit::timer_callback, this));

    this->declare_parameter ("lookahead_distance", 1.0);
    this->declare_parameter ("angle_p", 3.0);
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

    if (path_.poses.empty ()) {
        RCLCPP_WARN (this->get_logger (), "Path is empty");
        return;
    }

    // get the closest point on the path
    double min_distance  = std::numeric_limits<double>::max ();
    int    closest_index = -1;
    for (int index = 0; index < path_.poses.size (); index++) {
        double distance = std::hypot (
            path_.poses[index].pose.position.x - current_pose_.pose.position.x, path_.poses[index].pose.position.y - current_pose_.pose.position.y);
        if (distance < min_distance) {
            min_distance  = distance;
            closest_index = index;
        }
    }

    if (closest_index == -1) {
        RCLCPP_WARN (this->get_logger (), "No closest point found");
        return;
    }

    if (closest_index + 1 >= path_.poses.size ()) {
        RCLCPP_WARN (this->get_logger (), "Closest index is out of range");
        closest_index = path_.poses.size () - 2;
    }
    // get the lookahead point
    int lookahead_index = closest_index;
    for (int index = closest_index; index < path_.poses.size (); index++) {
        double distance = std::hypot (
            path_.poses[index].pose.position.x - current_pose_.pose.position.x, path_.poses[index].pose.position.y - current_pose_.pose.position.y);
        if (distance > lookahead_distance_) {
            break;
        }
        lookahead_index = index;
    }

    // calculate the angle to the lookahead point
    double dx = path_.poses[lookahead_index].pose.position.x - current_pose_.pose.position.x;
    double dy = path_.poses[lookahead_index].pose.position.y - current_pose_.pose.position.y;

    double current_yaw = std::asin (current_pose_.pose.orientation.z) * 2;
    double angle_diff  = std::atan2 (dy, dx) - current_yaw;
    double yaw_diff    = std::asin (path_.poses[lookahead_index].pose.orientation.z) * 2 - current_yaw;
    if (yaw_diff > M_PI) {
        yaw_diff -= 2.0 * M_PI;
    } else if (yaw_diff < -M_PI) {
        yaw_diff += 2.0 * M_PI;
    }

    int target_speed_index = closest_index;

    rclcpp::Time     time0 = path_.poses[target_speed_index].header.stamp;
    rclcpp::Time     time1 = path_.poses[target_speed_index + 1].header.stamp;
    rclcpp::Duration dt    = time1 - time0;
    if (dt.seconds () <= 0.0) {
        RCLCPP_WARN (this->get_logger (), "dt is zero or negative");
        return;
    }
    double speed = std::hypot (
                       path_.poses[target_speed_index + 1].pose.position.x - path_.poses[target_speed_index].pose.position.x,
                       path_.poses[target_speed_index + 1].pose.position.y - path_.poses[target_speed_index].pose.position.y) /
                   dt.seconds ();

    double yaw_speed = angle_p_ * yaw_diff;

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp    = this->now ();
    cmd_vel.header.frame_id = "base_link";
    cmd_vel.twist.linear.x  = speed * std::cos (angle_diff);
    cmd_vel.twist.linear.y  = speed * std::sin (angle_diff);
    cmd_vel.twist.angular.z = yaw_speed;
    cmd_vel_publisher_->publish (cmd_vel);

    geometry_msgs::msg::PoseStamped lookahead_point;
    lookahead_point.header.stamp    = this->now ();
    lookahead_point.header.frame_id = "map";
    lookahead_point.pose            = path_.poses[lookahead_index].pose;
    lookahead_publisher_->publish (lookahead_point);
}

}  // namespace pure_pursuit

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (pure_pursuit::pure_pursuit)
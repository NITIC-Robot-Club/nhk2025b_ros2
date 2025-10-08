#include "nhk2025b_path_planner/pose_array_publisher.hpp"

namespace pose_array_publisher {
pose_array_publisher::pose_array_publisher (const rclcpp::NodeOptions &options) : Node ("pose_array_publisher", options) {
    array_publisher           = this->create_publisher<geometry_msgs::msg::PoseArray> ("/behavior/goal_array", 1);
    pose_subscriber           = this->create_subscription<geometry_msgs::msg::PoseStamped> ("/behavior/goal_pose", 1, std::bind (&pose_array_publisher::pose_callback, this, std::placeholders::_1));
    publish_timing_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped> ("/behavior/e_drop_pose", 1, std::bind (&pose_array_publisher::publish_timing_callback, this, std::placeholders::_1));
}
void pose_array_publisher::pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    pose_array.poses.push_back (msg->pose);
}
void pose_array_publisher::publish_timing_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    publish_pose_array ();
    pose_array.poses.clear ();
}
void pose_array_publisher::publish_pose_array () {
    pose_array.header.stamp    = this->now ();
    pose_array.header.frame_id = "map";
    array_publisher->publish (pose_array);
    RCLCPP_INFO(this->get_logger (), "publish pose array with %d poses", (int)pose_array.poses.size ());
}
}  // namespace pose_array_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (pose_array_publisher::pose_array_publisher)
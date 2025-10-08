#ifndef __path_array_HPP__
#define __path_array_HPP__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>

#include <queue>
namespace pose_array_publisher {
class pose_array_publisher : public rclcpp::Node {
   public:
    pose_array_publisher (const rclcpp::NodeOptions& options);

   private:
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr      array_publisher;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr   pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr publish_timing_subscriber;

    void pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publish_timing_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publish_pose_array ();
    geometry_msgs::msg::PoseArray pose_array = geometry_msgs::msg::PoseArray();
};
}  // namespace pose_array_publisher

#endif  //__path_array_HPP__
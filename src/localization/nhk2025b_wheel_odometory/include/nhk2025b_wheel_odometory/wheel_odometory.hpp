#ifndef __wheel_odometory_hpp__
#define __wheel_odometory_hpp__

#include <nav_msgs/msg/odometry.hpp>
#include <nhk2025b_msgs/msg/swerve.hpp>
#include <rclcpp/rclcpp.hpp>

namespace wheel_odometory {
class wheel_odometory : public rclcpp::Node {
   public:
    wheel_odometory (const rclcpp::NodeOptions &options);

   private:
    double wheel_position, wheel_radius, current_x, current_y, current_z;

    rclcpp::Time last_time;

    void swerve_callback (const nhk2025b_msgs::msg::Swerve::SharedPtr msg);

    rclcpp::Subscription<nhk2025b_msgs::msg::Swerve>::SharedPtr swerve_subscriber;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       odom_publisher;
};
}  // namespace wheel_odometory

#endif  //__wheel_odometory_hpp__
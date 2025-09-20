#ifndef __velocity_smoother_hpp__
#define __velocity_smoother_hpp__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nhk2025b_msgs/msg/command.hpp>

namespace velocity_smoother {
class velocity_smoother : public rclcpp::Node {
   public:
    velocity_smoother (const rclcpp::NodeOptions& options);

   private:
    geometry_msgs::msg::TwistStamped::SharedPtr current_twist_;
    geometry_msgs::msg::TwistStamped  last_twist_;
    nhk2025b_msgs::msg::Command                 current_command_;

    double max_acceleration; // m/s^2

    void twist_auto_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void twist_controller_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void command_callback (const nhk2025b_msgs::msg::Command::SharedPtr msg);
    void timer_callback ();

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_auto_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_controller_sub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::Command>::SharedPtr      command_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr    twist_pub_;
    rclcpp::TimerBase::SharedPtr                                      timer_;
};
}  // namespace velocity_smoother

#endif  // __velocity_smoother_hpp__
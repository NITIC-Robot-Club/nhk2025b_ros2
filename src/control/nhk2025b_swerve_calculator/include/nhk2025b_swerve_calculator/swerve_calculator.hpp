
#ifndef __swerve_calculator_hpp__
#define __swerve_calculator_hpp__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nhk2025b_msgs/msg/command.hpp>
#include <nhk2025b_msgs/msg/swerve.hpp>

namespace swerve_calculator {
class swerve_calculator : public rclcpp::Node {
   public:
    swerve_calculator (const rclcpp::NodeOptions& options);

   private:
    void twist_auto_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void twist_controller_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void command_callback (const nhk2025b_msgs::msg::Command::SharedPtr msg);
    void timer_callback ();

    double wheel_radius;
    double robot_width, robot_length;

    geometry_msgs::msg::TwistStamped twist_auto;
    geometry_msgs::msg::TwistStamped twist_controller;
    nhk2025b_msgs::msg::Command      current_command_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_auto_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_controller_sub_;
    rclcpp::Publisher<nhk2025b_msgs::msg::Swerve>::SharedPtr          swerve_pub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::Command>::SharedPtr      command_sub_;
    rclcpp::TimerBase::SharedPtr                                      timer_;
};
}  // namespace swerve_calculator

#endif  //__swerve_calculator_hpp__
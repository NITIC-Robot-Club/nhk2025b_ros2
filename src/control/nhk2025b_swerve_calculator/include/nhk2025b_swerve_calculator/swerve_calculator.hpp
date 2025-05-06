
#ifndef __swerve_calculator_hpp__
#define __swerve_calculator_hpp__

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nhk2025b_msgs/msg/swerve.hpp>
#include <rclcpp/rclcpp.hpp>

namespace swerve_calculator {
class swerve_calculator : public rclcpp::Node {
   public:
    swerve_calculator (const rclcpp::NodeOptions& options);

   private:
    void twist_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    double wheel_radius;
    double wheel_position;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Publisher<nhk2025b_msgs::msg::Swerve>::SharedPtr          swerve_pub_;
};
}  // namespace swerve_calculator

#endif  //__swerve_calculator_hpp__
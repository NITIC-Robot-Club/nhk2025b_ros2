#ifndef __simulation_hpp__
#define __simulation_hpp__

#include <nhk2025b_msgs/msg/robot_status.hpp>
#include <nhk2025b_msgs/msg/swerve.hpp>
#include <rclcpp/rclcpp.hpp>

namespace simulation {
class simulation : public rclcpp::Node {
   public:
    simulation (const rclcpp::NodeOptions& options);

   private:
    void timer_callback ();
    void swerve_callback (const nhk2025b_msgs::msg::Swerve::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<nhk2025b_msgs::msg::Swerve>::SharedPtr      swerve_publisher_;
    rclcpp::Publisher<nhk2025b_msgs::msg::RobotStatus>::SharedPtr robot_status_publisher_;
    rclcpp::Subscription<nhk2025b_msgs::msg::Swerve>::SharedPtr   swerve_subscriber_;
};
}  // namespace simulation

#endif  //__simulation_hpp__
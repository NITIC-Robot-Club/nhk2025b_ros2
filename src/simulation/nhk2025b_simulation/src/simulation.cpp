
#include "nhk2025b_simulation/simulation.hpp"

namespace simulation {
simulation::simulation (const rclcpp::NodeOptions& options) : Node ("simulation", options) {
    swerve_publisher_       = this->create_publisher<nhk2025b_msgs::msg::Swerve> ("/swerve_cmd", 10);
    robot_status_publisher_ = this->create_publisher<nhk2025b_msgs::msg::RobotStatus> ("/robot_status", 10);
    swerve_subscriber_      = this->create_subscription<nhk2025b_msgs::msg::Swerve> (
        "/swerve_result", 10, std::bind (&simulation::swerve_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer (std::chrono::milliseconds (100), std::bind (&simulation::timer_callback, this));
}

void simulation::timer_callback () {}

void simulation::swerve_callback (const nhk2025b_msgs::msg::Swerve::SharedPtr msg) {
    swerve_publisher_->publish (*msg);
}
}  // namespace simulation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (simulation::simulation)
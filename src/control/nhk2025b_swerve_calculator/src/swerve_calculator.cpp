#include "nhk2025b_swerve_calculator/swerve_calculator.hpp"

namespace swerve_calculator {
swerve_calculator::swerve_calculator (const rclcpp::NodeOptions& options) : Node ("swerve_calculator", options) {
    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped> (
        "/cmd_vel", 10, std::bind (&swerve_calculator::twist_callback, this, std::placeholders::_1));
    swerve_pub_    = create_publisher<nhk2025b_msgs::msg::Swerve> ("/swerve/cmd", 10);
    wheel_position = this->declare_parameter<double> ("wheel_position", 0.62);
    wheel_radius   = this->declare_parameter<double> ("wheel_radius", 0.031);
}

void swerve_calculator::twist_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    double wheel_positions[4][2] = {
        {+wheel_position, +wheel_position},
        {-wheel_position, +wheel_position},
        {-wheel_position, -wheel_position},
        {+wheel_position, -wheel_position}
    };

    nhk2025b_msgs::msg::Swerve swerve_msg;
    swerve_msg.header.stamp    = this->now ();
    swerve_msg.header.frame_id = "base_link";

    for (int i = 0; i < 4; i++) {
        double x = msg->twist.linear.x;
        double y = msg->twist.linear.y;
        double z = msg->twist.angular.z;

        double position_x = wheel_positions[i][0];
        double position_y = wheel_positions[i][1];

        double vx, vy;
        if (x == 0 && y == 0 && z == 0) {
            vx = -position_y;
            vy = +position_x;

            swerve_msg.wheel_speed[i] = 0;
        } else {
            vx           = x - z * position_y;
            vy           = y + z * position_x;
            double v     = std::hypot (vx, vy);
            double omega = v / (2 * M_PI * wheel_radius);

            swerve_msg.wheel_speed[i] = omega * 60.f;
        }

        swerve_msg.wheel_angle[i] = atan2 (vy, vx);
    }
    swerve_pub_->publish (swerve_msg);
}
}  // namespace swerve_calculator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (swerve_calculator::swerve_calculator)
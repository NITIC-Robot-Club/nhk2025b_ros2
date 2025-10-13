#include "nhk2025b_swerve_calculator/swerve_calculator.hpp"

namespace swerve_calculator {
swerve_calculator::swerve_calculator (const rclcpp::NodeOptions& options) : Node ("swerve_calculator", options) {
    twist_auto_sub_       = create_subscription<geometry_msgs::msg::TwistStamped> ("/control/cmd_vel", 1, std::bind (&swerve_calculator::twist_auto_callback, this, std::placeholders::_1));
    twist_controller_sub_ = create_subscription<geometry_msgs::msg::TwistStamped> ("/controller/cmd_vel", 1, std::bind (&swerve_calculator::twist_controller_callback, this, std::placeholders::_1));
    command_sub_          = this->create_subscription<nhk2025b_msgs::msg::Command> ("/command", 1, std::bind (&swerve_calculator::command_callback, this, std::placeholders::_1));
    timer_                = create_wall_timer (std::chrono::milliseconds (10), std::bind (&swerve_calculator::timer_callback, this));
    swerve_pub_           = create_publisher<nhk2025b_msgs::msg::Swerve> ("/swerve/cmd", 1);
    robot_width           = this->declare_parameter<double> ("swerve_width", 0.8);
    robot_length          = this->declare_parameter<double> ("swerve_length", 0.6);
    wheel_radius          = this->declare_parameter<double> ("wheel_radius", 0.0325);
}

void swerve_calculator::command_callback (const nhk2025b_msgs::msg::Command::SharedPtr msg) {
    current_command_ = *msg;
}

void swerve_calculator::twist_auto_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    twist_auto = *msg;
}

void swerve_calculator::twist_controller_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    twist_controller = *msg;
}

void swerve_calculator::timer_callback () {
    double wheel_positions[4][2] = {
        {+robot_length / 2.0, +robot_width / 2.0},
        {-robot_length / 2.0, +robot_width / 2.0},
        {-robot_length / 2.0, -robot_width / 2.0},
        {+robot_length / 2.0, -robot_width / 2.0}
    };
    nhk2025b_msgs::msg::Swerve swerve_msg;
    geometry_msgs::msg::Twist  twist;

    if (current_command_.allow_automate) {
        twist = twist_auto.twist;
    } else {
        twist = twist_controller.twist;
    }

    for (int i = 0; i < 4; i++) {
        double x = twist.linear.x;
        double y = twist.linear.y;
        double z = twist.angular.z;

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
#include "nhk2025b_velocity_smoother/velocity_smoother.hpp"

namespace velocity_smoother {

velocity_smoother::velocity_smoother (const rclcpp::NodeOptions& options) : Node ("velocity_smoother", options) {
    twist_auto_sub_       = this->create_subscription<geometry_msgs::msg::TwistStamped> ("/control/cmd_vel", 1, std::bind (&velocity_smoother::twist_auto_callback, this, std::placeholders::_1));
    twist_controller_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped> ("/controller/cmd_vel", 1, std::bind (&velocity_smoother::twist_controller_callback, this, std::placeholders::_1));
    command_sub_          = this->create_subscription<nhk2025b_msgs::msg::Command> ("/command", 1, std::bind (&velocity_smoother::command_callback, this, std::placeholders::_1));
    twist_pub_            = this->create_publisher<geometry_msgs::msg::TwistStamped> ("/cmd_vel", 1);
    timer_                = create_wall_timer (std::chrono::milliseconds (10), std::bind (&velocity_smoother::timer_callback, this));
    this->declare_parameter ("max_acceleration", 1.0);
    this->declare_parameter ("allow_automate_always", false);
    bool allow_automate_always;
    this->get_parameter ("allow_automate_always", allow_automate_always);
    if (allow_automate_always) {
        current_command_.allow_automate = true;
    }
}

void velocity_smoother::twist_auto_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    if (current_command_.allow_automate) {
        current_twist_ = msg;
    }
}

void velocity_smoother::twist_controller_callback (const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    if (!current_command_.allow_automate) {
        current_twist_ = msg;
    }
}

void velocity_smoother::command_callback (const nhk2025b_msgs::msg::Command::SharedPtr msg) {
    current_command_ = *msg;
}

void velocity_smoother::timer_callback () {
    this->get_parameter ("max_acceleration", max_acceleration);
    geometry_msgs::msg::TwistStamped send_twist = geometry_msgs::msg::TwistStamped ();
    if (current_twist_) {
        send_twist = *current_twist_;
    } else {
    }
    double angle              = std::atan2 (send_twist.twist.linear.y, send_twist.twist.linear.x);
    double delta_t            = 0.01f;
    double last_speed         = std::hypot (last_twist_.twist.linear.x, last_twist_.twist.linear.y);
    double current_speed      = std::hypot (send_twist.twist.linear.x, send_twist.twist.linear.y);
    double acceleration       = (current_speed - last_speed) / delta_t;
    acceleration              = std::clamp (acceleration, -max_acceleration, max_acceleration);
    double speed              = last_speed + acceleration * delta_t;
    send_twist.twist.linear.x = speed * std::cos (angle);
    send_twist.twist.linear.y = speed * std::sin (angle);
    twist_pub_->publish (send_twist);
    last_twist_ = send_twist;
}

}  // namespace velocity_smoother

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (velocity_smoother::velocity_smoother)
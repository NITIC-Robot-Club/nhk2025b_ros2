#include <nhk2025b_simulation/arm_simulation.hpp>

#include <chrono>

namespace arm_simulation {

arm_simulation::arm_simulation (const rclcpp::NodeOptions &options) : Node ("arm_simulation", options) {
    pylon_arm_pub_ = this->create_publisher<nhk2025b_msgs::msg::PylonArm> ("/pylon_arm/result", rclcpp::QoS (10));
    e_arm_pub_     = this->create_publisher<nhk2025b_msgs::msg::EArm> ("/e_arm/result", rclcpp::QoS (10));
    box_arm_pub_   = this->create_publisher<nhk2025b_msgs::msg::BoxArm> ("/box_arm/result", rclcpp::QoS (10));

    pylon_arm_sub_ = this->create_subscription<nhk2025b_msgs::msg::PylonArm> ("/pylon_arm/cmd", rclcpp::QoS (10), std::bind (&arm_simulation::pylon_arm_callback, this, std::placeholders::_1));
    e_arm_sub_     = this->create_subscription<nhk2025b_msgs::msg::EArm> ("/e_arm/cmd", rclcpp::QoS (10), std::bind (&arm_simulation::e_arm_callback, this, std::placeholders::_1));
    box_arm_sub_   = this->create_subscription<nhk2025b_msgs::msg::BoxArm> ("/box_arm/cmd", rclcpp::QoS (10), std::bind (&arm_simulation::box_arm_callback, this, std::placeholders::_1));
    timer_         = this->create_wall_timer (std::chrono::milliseconds (100), std::bind (&arm_simulation::publish_arms, this));
}

void arm_simulation::pylon_arm_callback (const nhk2025b_msgs::msg::PylonArm::SharedPtr msg) {
    pylon_arm_target_ = *msg;
}

void arm_simulation::e_arm_callback (const nhk2025b_msgs::msg::EArm::SharedPtr msg) {
    e_arm_target_ = *msg;
}

void arm_simulation::box_arm_callback (const nhk2025b_msgs::msg::BoxArm::SharedPtr msg) {
    box_arm_target_ = *msg;
}

void arm_simulation::publish_arms () {
    for (size_t i = 0; i < 2; ++i) {
        last_pylon_arm_.expand[i] += (pylon_arm_target_.expand[i] - last_pylon_arm_.expand[i]) * 0.1;
        last_pylon_arm_.height[i] += (pylon_arm_target_.height[i] - last_pylon_arm_.height[i]) * 0.1;
    }
    pylon_arm_pub_->publish (last_pylon_arm_);

    last_e_arm_.expand += (e_arm_target_.expand - last_e_arm_.expand) * 0.1;
    last_e_arm_.get += (e_arm_target_.get - last_e_arm_.get) * 0.1;
    e_arm_pub_->publish (last_e_arm_);

    for (size_t i = 0; i < 2; ++i) {
        last_box_arm_.expand[i] += (box_arm_target_.expand[i] - last_box_arm_.expand[i]) * 0.1;
        last_box_arm_.height[i] += (box_arm_target_.height[i] - last_box_arm_.height[i]) * 0.1;
        last_box_arm_.hand_position[i] += (box_arm_target_.hand_position[i] - last_box_arm_.hand_position[i]) * 0.1;
    }
    box_arm_pub_->publish (last_box_arm_);
}

}  // namespace arm_simulation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (arm_simulation::arm_simulation)

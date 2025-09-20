#include "nhk2025b_joint_state_publisher/joint_state_publisher.hpp"

namespace joint_state_publisher {
joint_state_publisher::joint_state_publisher (const rclcpp::NodeOptions &options) : Node ("joint_state_publisher", options) {
    topicname = this->declare_parameter<std::string> ("topicname", "cmd");

    _joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState> ("joint_states", 1);
    pylon_arm_subscriber  = this->create_subscription<nhk2025b_msgs::msg::PylonArm> ("/pylon_arm/"+topicname, 1, std::bind (&joint_state_publisher::pylon_arm_callback, this, std::placeholders::_1));
    e_arm_subscriber     = this->create_subscription<nhk2025b_msgs::msg::EArm> ("/e_arm/"+topicname, 1, std::bind (&joint_state_publisher::e_arm_callback, this, std::placeholders::_1));
    box_arm_subscriber   = this->create_subscription<nhk2025b_msgs::msg::BoxArm> ("/box_arm/"+topicname, 1, std::bind (&joint_state_publisher::box_arm_callback, this, std::placeholders::_1));
    timer_                  = this->create_wall_timer (std::chrono::milliseconds (100), std::bind (&joint_state_publisher::timer_callback, this));
}

void joint_state_publisher::timer_callback () {
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.frame_id = "base_link";
    joint_state_msg.header.stamp = this->now ();
    joint_state_msg.name.resize (11);
    joint_state_msg.position.resize (11);
    joint_state_msg.velocity.resize (11);
    joint_state_msg.effort.resize (11);
    joint_state_msg.name[0]     = "c_nhk2025b_to_r_box_arm";
    if(box_arm_msg.expand.size() > 1) joint_state_msg.position[0] = box_arm_msg.expand[1];
    joint_state_msg.name[1]     = "r_box_arm_to_r_box_arm_height";
    if(box_arm_msg.height.size() > 1) joint_state_msg.position[1] = box_arm_msg.height[1];
    joint_state_msg.name[2]     = "r_box_arm_height_to_r_box_arm_strong";
    if(box_arm_msg.arm_position_strong.size() > 1) joint_state_msg.position[2] = box_arm_msg.arm_position_strong[1];
    joint_state_msg.name[3]     = "c_nhk2025b_to_l_box_arm";
    if(box_arm_msg.expand.size() > 1) joint_state_msg.position[3] = box_arm_msg.expand[0];
    joint_state_msg.name[4]     = "l_box_arm_to_l_box_arm_height";
    if(box_arm_msg.height.size() > 1) joint_state_msg.position[4] = box_arm_msg.height[0];
    joint_state_msg.name[5]     = "l_box_arm_height_to_l_box_arm_strong";
    if(box_arm_msg.arm_position_strong.size() > 0) joint_state_msg.position[5] = box_arm_msg.arm_position_strong[0];
    joint_state_msg.name[6]     = "c_nhk2025b_to_c_E_arm";
    joint_state_msg.position[6] = e_arm_msg.expand;
    joint_state_msg.name[7]     = "c_nhk2025b_to_l_pylon_arm_height";
    if(pylon_arm_msg.height.size() > 1) joint_state_msg.position[7] = pylon_arm_msg.height[0];
    joint_state_msg.name[8]     = "l_pylon_arm_height_to_l_pylon_arm";
    if(pylon_arm_msg.expand.size() > 1) joint_state_msg.position[8] = pylon_arm_msg.expand[0];
    joint_state_msg.name[9]     = "c_nhk2025b_to_r_pylon_arm_height";
    if(pylon_arm_msg.height.size() > 1) joint_state_msg.position[9] = pylon_arm_msg.height[1];
    joint_state_msg.name[10]    = "r_pylon_arm_height_to_r_pylon_arm";
    if(pylon_arm_msg.expand.size() > 1) joint_state_msg.position[10] = pylon_arm_msg.expand[1];
    _joint_state_publisher->publish (joint_state_msg);
}
void joint_state_publisher::pylon_arm_callback (const nhk2025b_msgs::msg::PylonArm::SharedPtr msg) {
    pylon_arm_msg = *msg;
}
void joint_state_publisher::e_arm_callback (const nhk2025b_msgs::msg::EArm::SharedPtr msg) {
    e_arm_msg = *msg;
}
void joint_state_publisher::box_arm_callback (const nhk2025b_msgs::msg::BoxArm::SharedPtr msg) {
    box_arm_msg = *msg;
}
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (joint_state_publisher::joint_state_publisher)
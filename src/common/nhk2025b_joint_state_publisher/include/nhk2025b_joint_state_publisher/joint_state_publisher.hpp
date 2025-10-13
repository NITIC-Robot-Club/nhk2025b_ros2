#ifndef __joint_state_publisher_HPP__
#define __joint_state_publisher_HPP__

#include <rclcpp/rclcpp.hpp>

#include <nhk2025b_msgs/msg/box_arm.hpp>
#include <nhk2025b_msgs/msg/e_arm.hpp>
#include <nhk2025b_msgs/msg/pylon_arm.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <queue>

namespace joint_state_publisher {
class joint_state_publisher : public rclcpp::Node {
   public:
    joint_state_publisher (const rclcpp::NodeOptions& options);

   private:
    void timer_callback ();
    void pylon_arm_callback (const nhk2025b_msgs::msg::PylonArm::SharedPtr msg);
    void e_arm_callback (const nhk2025b_msgs::msg::EArm::SharedPtr msg);
    void box_arm_callback (const nhk2025b_msgs::msg::BoxArm::SharedPtr msg);

    std::string                                                   topicname;
    nhk2025b_msgs::msg::PylonArm                                  pylon_arm_msg;
    nhk2025b_msgs::msg::EArm                                      e_arm_msg;
    nhk2025b_msgs::msg::BoxArm                                    box_arm_msg;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    _joint_state_publisher;
    rclcpp::Subscription<nhk2025b_msgs::msg::PylonArm>::SharedPtr pylon_arm_subscriber;
    rclcpp::Subscription<nhk2025b_msgs::msg::EArm>::SharedPtr     e_arm_subscriber;
    rclcpp::Subscription<nhk2025b_msgs::msg::BoxArm>::SharedPtr   box_arm_subscriber;
    rclcpp::TimerBase::SharedPtr                                  timer_;
};
}  // namespace joint_state_publisher

#endif  //__joint_state_publisher_HPP__
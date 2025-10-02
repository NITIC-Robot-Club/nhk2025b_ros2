#ifndef __arm_simulation_hpp__
#define __arm_simulation_hpp__

#include <rclcpp/rclcpp.hpp>

#include <nhk2025b_msgs/msg/box_arm.hpp>
#include <nhk2025b_msgs/msg/e_arm.hpp>
#include <nhk2025b_msgs/msg/pylon_arm.hpp>

namespace arm_simulation {
class arm_simulation : public rclcpp::Node {
   public:
    arm_simulation (const rclcpp::NodeOptions &options);

   private:
    rclcpp::Publisher<nhk2025b_msgs::msg::PylonArm>::SharedPtr pylon_arm_pub_;
    rclcpp::Publisher<nhk2025b_msgs::msg::EArm>::SharedPtr     e_arm_pub_;
    rclcpp::Publisher<nhk2025b_msgs::msg::BoxArm>::SharedPtr   box_arm_pub_;

    rclcpp::Subscription<nhk2025b_msgs::msg::PylonArm>::SharedPtr pylon_arm_sub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::EArm>::SharedPtr     e_arm_sub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::BoxArm>::SharedPtr   box_arm_sub_;

    nhk2025b_msgs::msg::PylonArm pylon_arm_target_;
    nhk2025b_msgs::msg::EArm     e_arm_target_;
    nhk2025b_msgs::msg::BoxArm   box_arm_target_;

    nhk2025b_msgs::msg::PylonArm last_pylon_arm_;
    nhk2025b_msgs::msg::EArm     last_e_arm_;
    nhk2025b_msgs::msg::BoxArm   last_box_arm_;

    rclcpp::TimerBase::SharedPtr timer_;

    void pylon_arm_callback (const nhk2025b_msgs::msg::PylonArm::SharedPtr msg);
    void e_arm_callback (const nhk2025b_msgs::msg::EArm::SharedPtr msg);
    void box_arm_callback (const nhk2025b_msgs::msg::BoxArm::SharedPtr msg);
    void publish_arms ();
};
}  // namespace arm_simulation

#endif  // __arm_simulation_hpp__
#ifndef __CANABLE_NODE_HPP__
#define __CANABLE_NODE_HPP__

#include "rclcpp/rclcpp.hpp"

#include "nhk2025b_msgs/msg/robot_status.hpp"
#include "nhk2025b_msgs/msg/swerve.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>

namespace canable {
class canable : public rclcpp::Node {
   public:
    canable (const rclcpp::NodeOptions &node_options);
    ~canable ();

   private:
    int  init_can_socket ();                                                 // Initialize CAN socket
    void read_can_socket ();                                                 // Read messages from CAN socket
    void swerve_callback (const nhk2025b_msgs::msg::Swerve::SharedPtr msg);  // Callback for swerve messages
    bool retry_open_can        = true;
    bool retry_write_can       = true;
    int  retry_write_count     = 0;
    int  max_retry_write_count = 5;
    int  can_socket_;
    bool swerve_flag_[4] = {false, false, false, false};

    struct sockaddr_can             addr_;
    struct ifreq                    ifr_;
    nhk2025b_msgs::msg::Swerve      swerve_now_;
    nhk2025b_msgs::msg::RobotStatus robot_status_;

    rclcpp::Publisher<nhk2025b_msgs::msg::RobotStatus>::SharedPtr robot_status_pub_;
    rclcpp::Publisher<nhk2025b_msgs::msg::Swerve>::SharedPtr      swerve_pub_;
    rclcpp::Subscription<nhk2025b_msgs::msg::Swerve>::SharedPtr   swerve_sub_;
};
}  // namespace canable

#endif  // __CANABLE_NODE_HPP__
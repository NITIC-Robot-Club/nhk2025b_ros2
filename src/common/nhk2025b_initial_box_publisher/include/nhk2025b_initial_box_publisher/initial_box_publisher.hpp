#ifndef __INITIAL_BOX_PUBLISHER_HPP__
#define __INITIAL_BOX_PUBLISHER_HPP__

#include "rclcpp/rclcpp.hpp"

#include "nhk2025b_msgs/msg/box_arm.hpp"
#include "nhk2025b_msgs/msg/box_array.hpp"
#include "nhk2025b_msgs/msg/command.hpp"
#include "std_msgs/msg/bool.hpp"

namespace initial_box_publisher {
class initial_box_publisher : public rclcpp::Node {
   public:
    initial_box_publisher (const rclcpp::NodeOptions &options);

   private:
    bool                                                       is_red = false;
    rclcpp::Publisher<nhk2025b_msgs::msg::BoxArray>::SharedPtr box_array_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr       is_red_subscriber_;
    rclcpp::TimerBase::SharedPtr                               timer_;

    void                         timer_callback ();
    void                         is_red_callback (const std_msgs::msg::Bool::SharedPtr msg);
    nhk2025b_msgs::msg::BoxArray get_initial_box_array ();
    void set_coat_box_array ();

    nhk2025b_msgs::msg::BoxArray             box_array;
    std::vector<std::pair<std::string, int>> box_names = {
        {"a", 0},
        {"a", 1},
        {"a", 2},
        {"a", 3},
        {"a", 4},
        {"a", 5},
        {"a", 6},
        {"a", 7},
        {"a", 8},
        {"a", 9},
        {"b", 0},
        {"b", 1},
        {"b", 2},
        {"b", 3},
        {"b", 4},
        {"c", 0},
        {"c", 1}
    };
};

}  // namespace initial_box_publisher

#endif  // __INITIAL_BOX_PUBLISHER_HPP__

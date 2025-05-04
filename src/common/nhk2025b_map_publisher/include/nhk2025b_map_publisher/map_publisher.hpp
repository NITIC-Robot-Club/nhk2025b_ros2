#ifndef __map_publisher_HPP__
#define __map_publisher_HPP__

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace map_publisher {
class map_publisher : public rclcpp::Node {
   public:
    map_publisher (const rclcpp::NodeOptions& options);

   private:
    void publish_map ();
    bool is_red;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr team_color_subsctiber_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace map_publisher

#endif  //__map_publisher_HPP__
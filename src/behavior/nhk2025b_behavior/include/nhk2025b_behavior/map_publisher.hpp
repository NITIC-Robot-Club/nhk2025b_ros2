#ifndef __map_publisher_HPP__
#define __map_publisher_HPP__

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace map_publisher {
    class map_publisher : public rclcpp::Node {
    public:
        map_publisher(const rclcpp::NodeOptions & options);
    private:
        float resolution_;
        void publish_map();
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}

#endif//__map_publisher_HPP__
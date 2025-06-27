#ifndef VISUALIZE_STL_NODE_HPP
#define VISUALIZE_STL_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace visualize_stl{
    class visualize_stl : public rclcpp::Node{
    public:
        explicit visualize_stl(const rclcpp::NodeOptions &node_options);
    private:
        void timer_callback();
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher;
        visualization_msgs::msg::Marker marker_msg;
        rclcpp::TimerBase::SharedPtr timer;
    };
}

#endif
#ifndef __visualize_swerve_HPP__
#define __visualize_swerve_HPP__

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nhk2025b_msgs/msg/swerve.hpp>

namespace visualize_swerve {
    class visualize_swerve : public rclcpp::Node {
    public:
        visualize_swerve(const rclcpp::NodeOptions & options);
    private:
        void swerve_callback(const nhk2025b_msgs::msg::Swerve::SharedPtr msg);
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
        rclcpp::Subscription<nhk2025b_msgs::msg::Swerve>::SharedPtr sub_;
    };
}

#endif//__visualize_swerve_HPP__

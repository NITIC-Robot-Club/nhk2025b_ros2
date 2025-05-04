#ifndef __visualize_swerve_HPP__
#define __visualize_swerve_HPP__

#include <nhk2025b_msgs/msg/swerve.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace visualize_swerve {
class visualize_swerve : public rclcpp::Node {
   public:
    visualize_swerve (const rclcpp::NodeOptions& options);

   private:
    float wheel_position;
    float wheel_radius;
    float last_angle[4];
    void swerve_callback (const nhk2025b_msgs::msg::Swerve::SharedPtr msg);
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Subscription<nhk2025b_msgs::msg::Swerve>::SharedPtr swerve_subscriber_;
};
}  // namespace visualize_swerve

#endif  //__visualize_swerve_HPP__

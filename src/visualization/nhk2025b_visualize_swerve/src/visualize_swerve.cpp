#include "nhk2025b_visualize_swerve/visualize_swerve.hpp"

namespace visualize_swerve {
    visualize_swerve::visualize_swerve(const rclcpp::NodeOptions & options)
        : Node("visualize_swerve",options) {
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/swerve", 10);
        swerve_subscriber_ = this->create_subscription<nhk2025b_msgs::msg::Swerve>("/swerve", 10, std::bind(&visualize_swerve::swerve_callback, this, std::placeholders::_1));
        wheel_position = this->declare_parameter<float>("wheel_position", 0.622);
        wheel_radius = this->declare_parameter<float>("wheel_radius", 0.062);
    }

    void visualize_swerve::swerve_callback(const nhk2025b_msgs::msg::Swerve::SharedPtr msg) {
        wheel_position = this->get_parameter("wheel_position").as_double();
        wheel_radius = this->get_parameter("wheel_radius").as_double();
        float wheel_positions[4][2] = {
            {+ wheel_position, + wheel_position},
            {- wheel_position, + wheel_position},
            {- wheel_position, - wheel_position},
            {+ wheel_position, - wheel_position}
        };
        visualization_msgs::msg::MarkerArray marker_array_;
        for(int i = 0; i < 4; i++) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = this->now();
            marker.ns = "swerve";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = wheel_positions[i][0];
            marker.pose.position.y = wheel_positions[i][1];
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = sin(msg->wheel_angle[i] / 2.);
            marker.pose.orientation.w = cos(msg->wheel_angle[i] / 2.);
            marker.scale.x = wheel_radius + msg->wheel_speed[i];
            marker.scale.y = wheel_radius;
            marker.scale.z = wheel_radius;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.lifetime = rclcpp::Duration(0, 0);
            marker_array_.markers.push_back(marker);

            visualization_msgs::msg::Marker text;
            text.header.frame_id = "base_link";
            text.header.stamp = this->now();
            text.ns = "swerve_angle_text";
            text.id = i;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.pose.position.x = wheel_positions[i][0];
            text.pose.position.y = wheel_positions[i][1];
            text.pose.position.z = 0.3;
            text.pose.orientation.x = 0.0;
            text.pose.orientation.y = 0.0;
            text.pose.orientation.z = 0.0;
            text.pose.orientation.w = 1.0;
            text.scale.z = 0.05;
            text.text = std::to_string(msg->wheel_angle[i]/M_PI) + "pi";
            if(abs(msg->wheel_angle[i] - last_angle[i]) > M_PI) {
                text.color.r = 1.0;
                text.color.g = 0.0;
                text.color.b = 0.0;
                text.color.a = 1.0;
            } else {
                text.color.r = 1.0;
                text.color.g = 1.0;
                text.color.b = 1.0;
                text.color.a = 1.0;
            }
            text.lifetime = rclcpp::Duration(0, 0);
            marker_array_.markers.push_back(text);
            last_angle[i] = msg->wheel_angle[i];
        }
        marker_publisher_->publish(marker_array_);
    }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(visualize_swerve::visualize_swerve)
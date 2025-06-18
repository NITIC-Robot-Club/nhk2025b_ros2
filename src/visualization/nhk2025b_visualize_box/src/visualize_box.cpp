#include "nhk2025b_visualize_box/visualize_box.hpp"

namespace visualize_box {

visualize_box::visualize_box(const rclcpp::NodeOptions &options)
: Node("visualize_box", options)
{
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization/box_array", 10);

    box_subscriber_ = this->create_subscription<nhk2025b_msgs::msg::BoxArray>(
        "/box_state", 10,
        std::bind(&visualize_box::box_callback, this, std::placeholders::_1));
}

void visualize_box::box_callback(const nhk2025b_msgs::msg::BoxArray::SharedPtr msg)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto &box : msg->boxes) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "boxes";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = box.pose;
        marker.scale = box.size;
        marker.pose.position.z += box.size.z / 2.0;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;
        marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        marker_array.markers.push_back(marker);
    }
    marker_publisher_->publish(marker_array);
}

}  // namespace visualize_box

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(visualize_box::visualize_box)
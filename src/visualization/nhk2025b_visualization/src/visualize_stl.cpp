#include "nhk2025b_visualization/visualize_stl.hpp"

namespace visualize_stl {
visualize_stl::visualize_stl (const rclcpp::NodeOptions &node_options) : rclcpp::Node ("viz_stl", node_options) {
    publisher = create_publisher<visualization_msgs::msg::Marker> ("output/maker", 0);

    marker_msg = visualization_msgs::msg::Marker ();

    marker_msg.header.frame_id = declare_parameter<std::string> ("frame_id", "object");

    marker_msg.ns     = declare_parameter<std::string> ("marker.namespace", "object");
    marker_msg.id     = declare_parameter<int> ("marker.id", 0);
    marker_msg.type   = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;

    marker_msg.scale.x = declare_parameter<double> ("marker.scale", 1.0);
    marker_msg.scale.y = marker_msg.scale.x;
    marker_msg.scale.z = marker_msg.scale.x;

    marker_msg.color.r = declare_parameter<double> ("marker.color.r", 1.0);
    marker_msg.color.g = declare_parameter<double> ("marker.color.g", 1.0);
    marker_msg.color.b = declare_parameter<double> ("marker.color.b", 1.0);
    marker_msg.color.a = declare_parameter<double> ("marker.color.a", 1.0);

    marker_msg.pose.position.x = declare_parameter<double> ("marker.position.x", 0.0);
    marker_msg.pose.position.y = declare_parameter<double> ("marker.position.y", 0.0);
    marker_msg.pose.position.z = declare_parameter<double> ("marker.position.z", 0.0);

    marker_msg.pose.orientation.x = declare_parameter<double> ("marker.orientation.x", 0.0);
    marker_msg.pose.orientation.y = declare_parameter<double> ("marker.orientation.y", 0.0);
    marker_msg.pose.orientation.z = declare_parameter<double> ("marker.orientation.z", 0.0);
    marker_msg.pose.orientation.w = declare_parameter<double> ("marker.orientation.w", 1.0);

    marker_msg.lifetime.sec     = declare_parameter<double> ("marker.lifetime.sec", 0.0);
    marker_msg.lifetime.nanosec = declare_parameter<double> ("marker.lifetime.nanosec", 0.0);

    marker_msg.frame_locked = false;

    marker_msg.mesh_resource               = "file://" + declare_parameter<std::string> ("marker.meshpath", "/path/to/object");
    marker_msg.mesh_use_embedded_materials = false;

    const auto period = declare_parameter<int> ("publish_rate_ms", 1000);
    timer             = create_wall_timer (std::chrono::milliseconds (period), std::bind (&visualize_stl::timer_callback, this));

    RCLCPP_INFO_STREAM (get_logger (), "ns:" << marker_msg.ns);
    RCLCPP_INFO_STREAM (get_logger (), "id:" << marker_msg.id);
    RCLCPP_INFO_STREAM (get_logger (), "file:" << marker_msg.mesh_resource);
    RCLCPP_INFO_STREAM (get_logger (), "Initialize Task Done");
}

void visualize_stl::timer_callback () {
    marker_msg.header.stamp = get_clock ()->now ();
    publisher->publish (marker_msg);
}

};  // namespace visualize_stl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (visualize_stl::visualize_stl)
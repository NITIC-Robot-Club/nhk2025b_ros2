#include "nhk2025b_simulation/box_state_simulation.hpp"

namespace box_state_simulation {
box_state_simulation::box_state_simulation(const rclcpp::NodeOptions &options)
    : Node("box_state_simulation", options){

    point_subscriber = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", rclcpp::QoS(10),
        std::bind(&box_state_simulation::point_callback, this, std::placeholders::_1));

    box_publisher = this->create_publisher<nhk2025b_msgs::msg::BoxArray>(
        "/box_state", rclcpp::QoS(10));
    timer = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&box_state_simulation::timer_callback, this));
    box_size_x = 0.5;
    box_size_y = 0.5;
    box_size_z = 0.5;
}

void box_state_simulation::point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    nhk2025b_msgs::msg::Box box;
    box.info.type = nhk2025b_msgs::msg::BoxInfo::A; // Example type
    box.info.id = current_box_array.boxes.size() + 1; // Incremental ID
    box.pose.position = msg->point;
    box.size.x = box_size_x;
    box.size.y = box_size_y;
    box.size.z = box_size_z;
    current_box_array.boxes.push_back(box);
}

void box_state_simulation::timer_callback() {
    box_publisher->publish(current_box_array);
}

}  // namespace box_state_simulation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(box_state_simulation::box_state_simulation)

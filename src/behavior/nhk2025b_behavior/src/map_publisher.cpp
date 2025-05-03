
#include "nhk2025b_behavior/map_publisher.hpp"

namespace map_publisher {
    map_publisher::map_publisher(const rclcpp::NodeOptions & options)
        : Node("map_publisher",options) {
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/behavior/map", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&map_publisher::publish_map, this));
        resolution_ = this->declare_parameter<float>("resolution", 0.05); // 5cm
        RCLCPP_INFO(this->get_logger(), "Map publisher initialized with resolution: %f", resolution_);
    }

    void map_publisher::publish_map() {
        nav_msgs::msg::OccupancyGrid map;
        resolution_ = this->get_parameter("resolution").as_double();
        map.header.stamp = this->now();
        map.header.frame_id = "map";
        map.info.resolution = resolution_;  // m
        map.info.width = 10.5 / resolution_; // 10m
        map.info.height = 5.25 / resolution_;  // 5m
        map.info.origin.position.x = 0.0;
        map.info.origin.position.y = 0.0;
        map.info.origin.position.z = 0.0;
        map.info.origin.orientation.w = 1.0;
        map.data.resize(map.info.width * map.info.height, -1);
        for(int i = 0; i < map.info.width; ++i) {
            for (int j = 0; j < map.info.height; ++j) {
                map.data[i + j * map.info.width] = 0;
            }
        }
        
        // 手前の壁
        // (0,0) ~ (0.015,5.25)
        for (int i = 0; i < map.info.width; ++i) {
            for (int j = 0; j < map.info.height; ++j) {
                if (i < 0.015 / resolution_) {
                    map.data[i + j * map.info.width] = 100;
                }
            }
        }

        // 左右の壁
        publisher_->publish(map);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(map_publisher::map_publisher)
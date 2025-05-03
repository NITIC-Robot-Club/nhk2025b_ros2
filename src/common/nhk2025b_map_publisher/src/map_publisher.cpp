
#include "nhk2025b_map_publisher/map_publisher.hpp"

namespace map_publisher {
    map_publisher::map_publisher(const rclcpp::NodeOptions & options)
        : Node("map_publisher",options) {
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/behavior/map", 10);
        team_color_subsctiber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/is_red", 10,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                is_red = msg->data;
            });
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&map_publisher::publish_map, this));
        this->declare_parameter<float>("resolution", 0.05); // 5cm
    }

    void map_publisher::publish_map() {
        nav_msgs::msg::OccupancyGrid map;
        float resolution_ = this->get_parameter("resolution").as_double();
        map.header.stamp = this->now();
        map.header.frame_id = "map";
        map.info.resolution = resolution_;  // m
        map.info.width = 10.8 / resolution_; // 10m
        map.info.height = 5.4 / resolution_;  // 5m
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
        
        for (int y = 0; y < map.info.width; ++y) {
            for (int x = 0; x < map.info.height; ++x) {
                // 手前 (0,0) ~ (0.15,5.25)
                if (y < 0.15 / resolution_) {
                    map.data[y + x * map.info.width] = 100;
                }
                if (is_red) {
                    // 左 (0,5.25)~(10.8,5.4)
                    if (x > 5.25 / resolution_) {
                        map.data[y + x * map.info.width] = 100;
                    }
                    // 奥 (10.65,5.4)~(10.8,0.6)
                    if (y > 10.65 / resolution_ && x > 0.6 / resolution_) {
                        map.data[y + x * map.info.width] = 100;
                    }
                    // 共有 (6.95,0)~(10.8,0.6)
                    if (y > 6.95 / resolution_ && x < 0.6 / resolution_) {
                        map.data[y + x * map.info.width] = 100;
                    }
                } else {
                    // 右 (0,0)~(10.8, 0.015)
                    if (x < 0.015 / resolution_) {
                        map.data[y + x * map.info.width] = 100;
                    }
                    // 奥 (10.65,0)~(10.8,4.8)
                    if (y > 10.65 / resolution_ && x < 4.8 / resolution_) {
                        map.data[y + x * map.info.width] = 100;
                    }
                    // 共有 (6.95,5.4)~(10.8,4.8)
                    if (y > 6.95 / resolution_ && x > 4.8 / resolution_) {
                        map.data[y + x * map.info.width] = 100;
                    }
                }
            }
        }

        publisher_->publish(map);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(map_publisher::map_publisher)
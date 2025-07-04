
#include "nhk2025b_map_publisher/map_publisher.hpp"

namespace map_publisher {
map_publisher::map_publisher (const rclcpp::NodeOptions& options) : Node ("map_publisher", options) {
    publisher_      = this->create_publisher<nav_msgs::msg::OccupancyGrid> ("/behavior/map", 1);
    box_subscriber_ = this->create_subscription<nhk2025b_msgs::msg::BoxArray> (
        "/box_state", 1, std::bind (&map_publisher::box_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer (std::chrono::milliseconds (1000), std::bind (&map_publisher::publish_map, this));
    this->declare_parameter<double> ("resolution", 0.05);  // 5cm
    this->declare_parameter<bool> ("is_red", false);
    this->get_parameter ("is_red", is_red);  // bool
    if (is_red) {
        for (int i = 0; i < 5; i++) {
            // 赤のときはfield_dataのyを左右反転にする
            double field_data_y[2];
            field_data_y[0]  = field_data[i][2];
            field_data_y[1]  = field_data[i][3];
            field_data[i][2] = 5.4 - field_data_y[1];
            field_data[i][3] = 5.4 - field_data_y[0];
        }
    }
}

void map_publisher::publish_map () {
    nav_msgs::msg::OccupancyGrid map;
    this->get_parameter ("resolution", resolution_);  // float
    map.header.stamp           = this->now ();
    map.header.frame_id        = "map";
    map.info.resolution        = resolution_;         // m
    map.info.width             = 10.8 / resolution_;  // 10m
    map.info.height            = 5.4 / resolution_;   // 5m
    map.info.origin.position.x = -0.15;
    map.info.origin.position.y = -0.15;
    if (is_red) {
        map.info.origin.position.y = -5.3;
    }
    map.info.origin.position.z    = 0.0;
    map.info.origin.orientation.w = 1.0;
    map.data.resize (map.info.width * map.info.height, -1);

    for (int i = 0; i < map.info.width; ++i) {
        for (int j = 0; j < map.info.height; ++j) {
            map.data[i + j * map.info.width] = 0;
        }
    }

    for (const auto& box : boxes.boxes) {
        // 1. Box情報の取得
        double center_x = box.pose.position.x;
        double center_y = box.pose.position.y;
        double size_x   = box.size.x;
        double size_y   = box.size.y;
        double yaw      = nhk2025b_utils::get_yaw_2d (box.pose.orientation);

        // 2. 回転行列（逆回転）を用意
        double cos_yaw = std::cos (-yaw);
        double sin_yaw = std::sin (-yaw);

        // 3. BoxのAABBでループ（回転を考慮した描画領域を最小化するならここ工夫してもOK）
        int min_x = (center_x - size_x / 2.0 - map.info.origin.position.x) / resolution_;
        int max_x = (center_x + size_x / 2.0 - map.info.origin.position.x) / resolution_;
        int min_y = (center_y - size_y / 2.0 - map.info.origin.position.y) / resolution_;
        int max_y = (center_y + size_y / 2.0 - map.info.origin.position.y) / resolution_;

        for (int mx = min_x; mx <= max_x; ++mx) {
            for (int my = min_y; my <= max_y; ++my) {
                // マップ範囲チェック
                if (mx < 0 || mx >= map.info.width || my < 0 || my >= map.info.height) continue;

                // マップ座標をワールド座標に変換
                double wx = mx * resolution_ + map.info.origin.position.x;
                double wy = my * resolution_ + map.info.origin.position.y;

                // 4. Box中心に対する相対座標に変換
                double dx = wx - center_x;
                double dy = wy - center_y;

                // 5. Box座標系に変換（yawの逆回転）
                double local_x = cos_yaw * dx - sin_yaw * dy;
                double local_y = sin_yaw * dx + cos_yaw * dy;

                // 6. Box内部か判定
                if (std::abs (local_x) < size_x / 2.0 && std::abs (local_y) < size_y / 2.0) {
                    map.data[mx + my * map.info.width] = 100;
                }
            }
        }
    }

    for (int y = 0; y < map.info.height; ++y) {
        for (int x = 0; x < map.info.width; ++x) {
            double wx = x * resolution_ + map.info.origin.position.x;
            double wy = y * resolution_ + map.info.origin.position.y;
            for (int i = 0; i < 5; ++i) {
                if (field_data[i][0] <= wx && wx <= field_data[i][1] && field_data[i][2] <= wy && wy <= field_data[i][3]) {
                    map.data[y * map.info.width + x] = 100;
                }
            }
        }
    }

    publisher_->publish (map);
}

void map_publisher::box_callback (const nhk2025b_msgs::msg::BoxArray::SharedPtr msg) {
    boxes = *msg;
    publish_map ();
}
}  // namespace map_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (map_publisher::map_publisher)
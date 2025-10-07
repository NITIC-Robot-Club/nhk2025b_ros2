#include "nhk2025b_visualization/visualize_path_collision.hpp"

namespace visualize_path_collision {

visualize_path_collision::visualize_path_collision (const rclcpp::NodeOptions &options) : Node ("visualize_path_collision", options) {
    path_sub_        = this->create_subscription<nav_msgs::msg::Path> ("/planning/path", 1, std::bind (&visualize_path_collision::path_callback, this, std::placeholders::_1));
    map_sub_         = this->create_subscription<nav_msgs::msg::OccupancyGrid> ("/behavior/map", 1, std::bind (&visualize_path_collision::map_callback, this, std::placeholders::_1));
    marker_pub_      = this->create_publisher<visualization_msgs::msg::MarkerArray> ("/visualization/path_collision", 1);
    timer_           = this->create_wall_timer (std::chrono::milliseconds (100), std::bind (&visualize_path_collision::timer_callback, this));
    robot_width_sub_ = this->create_subscription<std_msgs::msg::Float32> ("/robot_width", 1, [this] (const std_msgs::msg::Float32::SharedPtr msg) { robot_width = msg->data; });

    declare_parameter ("robot_length", 0.6);
}

void visualize_path_collision::path_callback (const nav_msgs::msg::Path::SharedPtr msg) {
    path = *msg;
}

void visualize_path_collision::timer_callback () {
    robot_length                 = get_parameter ("robot_length").as_double ();
    double wheel_positions[5][2] = {
        {+robot_length / 2.0, +robot_width / 2.0},
        {-robot_length / 2.0, +robot_width / 2.0},
        {-robot_length / 2.0, -robot_width / 2.0},
        {+robot_length / 2.0, -robot_width / 2.0},
        {+robot_length / 2.0, +robot_width / 2.0}
        // 最後の点は最初の点と同じにする
    };

    visualization_msgs::msg::MarkerArray marker_array;
    // MarkerArrayとしてpublish
    marker_array.markers.clear ();
    int id = 0;
    for (const auto &pose : path.poses) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp    = this->now ();
        marker.ns              = "path_collision";
        marker.id              = id++;
        marker.type            = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action          = visualization_msgs::msg::Marker::ADD;
        marker.scale.x         = 0.01;
        if (is_hit (std::make_shared<geometry_msgs::msg::PoseStamped> (pose))) {
            marker.color.r = 1.0;  // 赤色
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        marker.color.a  = 1.0;  // 不透明
        marker.lifetime = rclcpp::Duration::from_seconds (0.3);
        // ロボットの四隅の点を追加
        for (const auto &pos : wheel_positions) {
            geometry_msgs::msg::Point point;
            point.x = pose.pose.position.x + pos[0] * cos (nhk2025b_utils::get_yaw_2d (pose.pose.orientation)) - pos[1] * sin (nhk2025b_utils::get_yaw_2d (pose.pose.orientation));
            point.y = pose.pose.position.y + pos[0] * sin (nhk2025b_utils::get_yaw_2d (pose.pose.orientation)) + pos[1] * cos (nhk2025b_utils::get_yaw_2d (pose.pose.orientation));
            point.z = pose.pose.position.z;  // Z軸はそのまま
            marker.points.push_back (point);
        }
        marker_array.markers.push_back (marker);
    }
    marker_pub_->publish (marker_array);
}

void visualize_path_collision::map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map = *msg;
}

bool visualize_path_collision::is_hit (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (map.data.empty ()) return false;

    const double x = msg->pose.position.x;
    const double y = msg->pose.position.y;

    const double origin_x   = map.info.origin.position.x;
    const double origin_y   = map.info.origin.position.y;
    const double resolution = map.info.resolution;
    const int    width      = map.info.width;
    const int    height     = map.info.height;

    // yaw角度の取得（2D平面想定）
    double yaw = nhk2025b_utils::get_yaw_2d (msg->pose.orientation);

    // ロボットの四隅の点（ロボット中心が基準）
    std::vector<std::pair<double, double>> corners;
    for (int dx : {-1, 1}) {
        for (int dy : {-1, 1}) {
            double local_x = dx * robot_length / 2.0;
            double local_y = dy * robot_width / 2.0;

            // 回転＋平行移動（ロボット中心→地図座標）
            double world_x = x + std::cos (yaw) * local_x - std::sin (yaw) * local_y;
            double world_y = y + std::sin (yaw) * local_x + std::cos (yaw) * local_y;

            corners.emplace_back (world_x, world_y);
        }
    }

    // 四隅すべてを OccupancyGrid でチェック
    for (const auto &[px, py] : corners) {
        int col = static_cast<int> ((px - origin_x) / resolution);
        int row = static_cast<int> ((py - origin_y) / resolution);

        if (col < 0 || col >= width || row < 0 || row >= height) {
            return true;  // 範囲外＝危険とみなす
        }

        int    index = row * width + col;
        int8_t cell  = map.data[index];

        if (cell >= 50) {
            return true;  // 衝突
        }
    }

    return false;  // すべて安全
}

}  // namespace visualize_path_collision
// namespace visualize_path_collision

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (visualize_path_collision::visualize_path_collision)
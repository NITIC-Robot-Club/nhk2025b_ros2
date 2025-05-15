#include "nhk2025b_footprint_publisher/footprint_publisher.hpp"

namespace footprint_publisher {

footprint_publisher::footprint_publisher (const rclcpp::NodeOptions &options) : Node ("footprint_publisher", options) {
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/localization/current_pose", 10, std::bind (&footprint_publisher::pose_callback, this, std::placeholders::_1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid> (
        "/behavior/map", 10, std::bind (&footprint_publisher::map_callback, this, std::placeholders::_1));
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray> ("/visualization/robot_footprint", 10);

    // Robot dimensions (in meters)
    declare_parameter ("robot_width", 1.0);
    declare_parameter ("robot_length", 1.0);
    declare_parameter ("history", 300);
}

void footprint_publisher::pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // パラメータ取得
    robot_width  = get_parameter ("robot_width").as_double ();
    robot_length = get_parameter ("robot_length").as_double ();
    history      = get_parameter ("history").as_int ();
    if (history < 1) history = 1;

    // Marker作成
    visualization_msgs::msg::Marker marker;
    marker.header  = msg->header;
    marker.ns      = "robot_footprint";
    marker.id      = marker_history.size ();  // 毎回変化でよい（古いのは消す）
    marker.type    = visualization_msgs::msg::Marker::CUBE;
    marker.action  = visualization_msgs::msg::Marker::ADD;
    marker.pose    = msg->pose;
    marker.scale.x = robot_length;
    marker.scale.y = robot_width;
    marker.scale.z = 0.01;

    if (is_hit (msg)) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        // RCLCPP_WARN (get_logger(), "Robot footprint is in collision!");
    } else {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    }
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_seconds (0.5);  // 古いmarkerは自然に消す or 明示削除も可能

    // 履歴追加
    marker_history.push_back (marker);
    if (marker_history.size () > history) {
        marker_history.pop_front ();
    }

    visualization_msgs::msg::MarkerArray marker_array;
    // MarkerArrayとしてpublish
    marker_array.markers.clear ();
    int id = 0;
    for (auto &m : marker_history) {
        m.id = id++;  // IDをリセットして重複しないようにする
        if (id == marker_history.size ()) {
            m.pose.position.z = 0.1;  // 最後のマーカーは少し上に
            m.color.a         = 1.0;  // 不透明にする
        } else if (m.color.b == 1.0) {
            m.color.g         = 1.0;
            m.color.b         = 0.0;
            m.color.a         = 0.01;  // 半透明にする
            m.pose.position.z = 0.0;   // 地面に
        } else {
            m.color.a         = 0.1;  // 半透明にする
            m.pose.position.z = 0.0;  // 地面に
        }
        marker_array.markers.push_back (m);
    }

    marker_pub_->publish (marker_array);
}

void footprint_publisher::map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map = *msg;
}

bool footprint_publisher::is_hit (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (map.data.empty ()) return false;

    const double x = msg->pose.position.x;
    const double y = msg->pose.position.y;

    const double origin_x   = map.info.origin.position.x;
    const double origin_y   = map.info.origin.position.y;
    const double resolution = map.info.resolution;
    const int    width      = map.info.width;
    const int    height     = map.info.height;

    // yaw角度の取得（2D平面想定）
    double yaw = std::asin (msg->pose.orientation.z) * 2;

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

}  // namespace footprint_publisher
// namespace footprint_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (footprint_publisher::footprint_publisher)
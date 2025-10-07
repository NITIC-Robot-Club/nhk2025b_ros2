#include "nhk2025b_visualization/visualize_box.hpp"

namespace visualize_box {

visualize_box::visualize_box (const rclcpp::NodeOptions &options) : Node ("visualize_box", options) {
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray> ("/visualization/box", 10);

    box_subscriber_ = this->create_subscription<nhk2025b_msgs::msg::BoxArray> ("/simulation/box_state", 10, std::bind (&visualize_box::box_callback, this, std::placeholders::_1));
}

void visualize_box::box_callback (const nhk2025b_msgs::msg::BoxArray::SharedPtr msg) {
    visualization_msgs::msg::MarkerArray marker_array;
    int                                  id = 0;
    for (const auto &box : msg->boxes) {
        // --- 既存のCUBE ---
        visualization_msgs::msg::Marker cube;
        cube.header.frame_id = "map";
        cube.header.stamp    = this->now ();
        cube.ns              = "boxes";
        cube.id              = id++;
        cube.type            = visualization_msgs::msg::Marker::CUBE;
        cube.action          = visualization_msgs::msg::Marker::ADD;
        cube.pose            = box.pose;
        cube.scale           = box.size;
        cube.pose.position.z += box.size.z / 2.0;
        cube.color.r  = 1.0f;
        cube.color.g  = 1.0f;
        cube.color.b  = 1.0f;
        cube.color.a  = 1.0f;
        cube.lifetime = rclcpp::Duration::from_seconds (0.2);
        marker_array.markers.push_back (cube);

        visualization_msgs::msg::Marker edge;
        edge.header   = cube.header;
        edge.ns       = "box_edges";
        edge.id       = id++;
        edge.type     = visualization_msgs::msg::Marker::LINE_LIST;
        edge.action   = visualization_msgs::msg::Marker::ADD;
        edge.scale.x  = 0.03;
        edge.lifetime = rclcpp::Duration::from_seconds (0.2);

        double                    x = box.size.x / 2.0;
        double                    y = box.size.y / 2.0;
        double                    z = box.size.z;
        geometry_msgs::msg::Point p[8];
        p[0].x = x;
        p[0].y = y;
        p[0].z = 0;
        p[1].x = -x;
        p[1].y = y;
        p[1].z = 0;
        p[2].x = -x;
        p[2].y = -y;
        p[2].z = 0;
        p[3].x = x;
        p[3].y = -y;
        p[3].z = 0;
        p[4].x = x;
        p[4].y = y;
        p[4].z = z;
        p[5].x = -x;
        p[5].y = y;
        p[5].z = z;
        p[6].x = -x;
        p[6].y = -y;
        p[6].z = z;
        p[7].x = x;
        p[7].y = -y;
        p[7].z = z;

        int edges_idx[12][2] = {
            {0, 1},
            {1, 2},
            {2, 3},
            {3, 0},
            {4, 5},
            {5, 6},
            {6, 7},
            {7, 4},
            {0, 4},
            {1, 5},
            {2, 6},
            {3, 7}
        };

        tf2::Quaternion q;
        tf2::fromMsg (box.pose.orientation, q);

        for (auto &e : edges_idx) {
            geometry_msgs::msg::Point p1 = p[e[0]], p2 = p[e[1]];

            tf2::Vector3 v1 (p1.x, p1.y, p1.z), v2 (p2.x, p2.y, p2.z);
            v1 = tf2::quatRotate (q, v1);
            v2 = tf2::quatRotate (q, v2);

            // 中心から外向き法線方向へ1.5mmオフセット
            tf2::Vector3 center ((v1.x () + v2.x ()) / 2.0, (v1.y () + v2.y ()) / 2.0, (v1.z () + v2.z ()) / 2.0);
            tf2::Vector3 dir = center.normalized () * 0.0015;  // 1.5mm
            v1 += dir;
            v2 += dir;

            p1.x = v1.x () + box.pose.position.x;
            p1.y = v1.y () + box.pose.position.y;
            p1.z = v1.z () + box.pose.position.z;
            p2.x = v2.x () + box.pose.position.x;
            p2.y = v2.y () + box.pose.position.y;
            p2.z = v2.z () + box.pose.position.z;

            // 辺の長さをmmに変換
            double length = std::sqrt (std::pow (p2.x - p1.x, 2) + std::pow (p2.y - p1.y, 2) + std::pow (p2.z - p1.z, 2)) * 1000.0;

            // 色設定
            std_msgs::msg::ColorRGBA c;
            if (std::abs (length - 200.0) < 1e-3) {
                c.r = 1.0;
                c.g = 0.0;
                c.b = 1.0;
            }  // pink
            else if (std::abs (length - 300.0) < 1e-3) {
                c.r = 1.0;
                c.g = 1.0;
                c.b = 0.0;
            }  // yellow
            else if (std::abs (length - 400.0) < 1e-3) {
                c.r = 0.5;
                c.g = 1.0;
                c.b = 1.0;
            }  // light blue
            else if (std::abs (length - 500.0) < 1e-3) {
                c.r = 0.5;
                c.g = 1.0;
                c.b = 0.5;
            }  // light green
            else if (std::abs (length - 800.0) < 1e-3) {
                c.r = 1.0;
                c.g = 1.0;
                c.b = 1.0;
            }  // white
            else if (std::abs (length - 1000.0) < 1e-3) {
                c.r = 0.0;
                c.g = 0.0;
                c.b = 0.0;
            }  // black
            else {
                c.r = 1.0;
                c.g = 0.0;
                c.b = 0.0;
            }  // fallback red
            c.a = 1.0;

            // 頂点と色を追加
            edge.points.push_back (p1);
            edge.points.push_back (p2);
            edge.colors.push_back (c);
            edge.colors.push_back (c);
        }
        marker_array.markers.push_back (edge);
    }
    marker_publisher_->publish (marker_array);
}
}  // namespace visualize_box

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (visualize_box::visualize_box)
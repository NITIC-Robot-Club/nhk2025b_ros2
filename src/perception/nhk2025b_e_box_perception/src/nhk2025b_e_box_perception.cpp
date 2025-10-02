#include "nhk2025b_e_box_perception.hpp"

namespace e_box_perception {

e_box_perception::e_box_perception (const rclcpp::NodeOptions& options) : rclcpp::Node ("e_box_perception", options) {
    box_publisher_     = this->create_publisher<nhk2025b_msgs::msg::BoxArray> ("/box_state", rclcpp::QoS (10));
    pose_subscriber_   = this->create_subscription<geometry_msgs::msg::PoseStamped> ("/localization/current_pose", 1, std::bind (&e_box_perception::pose_callback, this, std::placeholders::_1));
    lidar_subscriber_  = this->create_subscription<sensor_msgs::msg::PointCloud2> ("/sensor/lidar", 1, std::bind (&e_box_perception::lidar_callback, this, std::placeholders::_1));
    is_red_subscriber_ = this->create_subscription<std_msgs::msg::Bool> ("/is_red", 1, std::bind (&e_box_perception::is_red_callback, this, std::placeholders::_1));
    iter               = this->declare_parameter<int> ("iter", 100);
    distance_threshold = this->declare_parameter<double> ("distance_threshold", 0.025);
    normal_distance    = this->declare_parameter<double> ("normal_distance", 0.5);
}

void e_box_perception::pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    current_pose_ = *pose;
}

void e_box_perception::is_red_callback (const std_msgs::msg::Bool::SharedPtr is_red) {
    is_red_ = is_red->data;
    if (is_red_) {  // 赤コートのときの検出範囲
        min_x = 0.025;
        max_x = 5.0;
        min_y = 0.525;
        max_y = 1.025;
    } else {  // 青コートのときの検出範囲
        min_x = 0.025;
        max_x = 5.0;
        min_y = 3.7;
        max_y = 5.2;
    }
}

void e_box_perception::lidar_callback (const sensor_msgs::msg::PointCloud2::SharedPtr lidar) {
    std::vector<e_box_perception::Point> points          = cloud_to_points (*lidar);                              // 点群データをPointの配列に変換
    std::vector<e_box_perception::Point> filtered_points = filtering_points (points);                             // 検出範囲に入っている点のみ抽出
    e_box_perception::Line               line            = ransac (filtered_points);                              // RANSACで直線検出
    e_box_perception::Point              centre_of_line  = line_centre (line);                                    // 直線の中心点を計算
    e_box_perception::Point              centre_of_box   = normal_point (line, centre_of_line, normal_distance);  // 直線の法線方向にnormal_distanceだけ離れた点を計算
    nhk2025b_msgs::msg::Box              box;
    box.info.type       = nhk2025b_msgs::msg::BoxInfo::E;
    box.info.id         = box_array_.boxes.size () + 1;
    box.pose.position.x = centre_of_box.x;
    box.pose.position.y = centre_of_box.y;
    box.pose.position.z = 0.0;
    box.size.x          = 0.3;
    box.size.y          = 1.0;
    box.size.z          = 0.3;
    box_array_.boxes.push_back (box);
}

std::vector<e_box_perception::Point> e_box_perception::cloud_to_points (const sensor_msgs::msg::PointCloud2& cloud) {
    std::vector<e_box_perception::Point> points;
    const uint8_t*                       data = cloud.data.data ();
    for (size_t i = 0; i < cloud.width * cloud.height; i++) {
        float x = *reinterpret_cast<const float*> (data + cloud.point_step * i + 0);
        float y = *reinterpret_cast<const float*> (data + cloud.point_step * i + 4);
        points.push_back ({x, y});
    }
    return points;
}

std::vector<e_box_perception::Point> e_box_perception::filtering_points (std::vector<e_box_perception::Point> data) {
    std::vector<e_box_perception::Point> filtered_points;
    for (const auto& point : data) {
        if (point.x >= min_x && point.x <= max_x && point.y >= min_y && point.y <= max_y) {
            filtered_points.push_back (point);
        }
    }
    return filtered_points;
}

e_box_perception::Point e_box_perception::line_centre (e_box_perception::Line line) {
    Point centre;
    centre.x = (line.start.x + line.end.x) / 2.0;
    centre.y = (line.start.y + line.end.y) / 2.0;
    return centre;
}

e_box_perception::Point e_box_perception::normal_point (e_box_perception::Line line, e_box_perception::Point point, double normal_distance) {
    double line_angle = atan2 (line.end.y - line.start.y, line.end.x - line.start.x);

    double normal_angle;
    if (is_red_) {
        normal_angle = line_angle + M_PI / 2.0;
    } else {
        normal_angle = line_angle - M_PI / 2.0;
    }
    e_box_perception::Point normal_pt;
    normal_pt.x = point.x + normal_distance * cos (normal_angle);
    normal_pt.y = point.y + normal_distance * sin (normal_angle);

    if (normal_pt.x >= min_x && normal_pt.x <= max_x && normal_pt.y >= min_y && normal_pt.y <= max_y) {
        return normal_pt;
    } else {
        normal_angle = is_red_ ? line_angle - M_PI / 2.0 : line_angle + M_PI / 2.0;
        normal_pt.x  = point.x + normal_distance * cos (normal_angle);
        normal_pt.y  = point.y + normal_distance * sin (normal_angle);

        if (normal_pt.x >= min_x && normal_pt.x <= max_x && normal_pt.y >= min_y && normal_pt.y <= max_y) {
            return normal_pt;
        } else {
            return point;
        }
    }
}

e_box_perception::Line e_box_perception::ransac (std::vector<e_box_perception::Point> data) {
    if (data.size () == 0) {
        Line err_line;
        err_line.a     = 0.0f;
        err_line.b     = 0.0f;
        err_line.start = {0.0f, 0.0f};
        err_line.end   = {0.0f, 0.0f};
        return err_line;
    }
    int best_inliers   = 0;
    iter               = this->get_parameter ("iter").as_int ();
    distance_threshold = this->get_parameter ("distance_threshold").as_double ();
    Line               best_line;
    std::vector<Point> best_inlier_points;

    for (int i = 0; i < iter; i++) {
        Point p1 = data[rand () % data.size ()];
        Point p2 = data[rand () % data.size ()];
        if (abs (p1.x - p2.x) < 1e-6) continue;
        double             a       = (p1.y - p2.y) / (p1.x - p2.x);
        double             b       = p1.y - a * p1.x;
        int                inliers = 0;
        std::vector<Point> inlier_points;
        for (size_t j = 0; j < data.size (); j++) {
            double d = fabs (a * data[j].x - data[j].y + b) / sqrt (a * a + 1);
            if (d <= distance_threshold) {
                inliers++;
                inlier_points.push_back (data[j]);
            }
        }

        if (inliers > best_inliers && !inlier_points.empty ()) {
            std::vector<Point> temp_inliers = inlier_points;
            std::sort (temp_inliers.begin (), temp_inliers.end (), [] (const Point& a, const Point& b) { return a.x < b.x; });

            Point line_start = temp_inliers.front ();
            Point line_end   = temp_inliers.back ();

            double line_length = sqrt (pow (line_end.x - line_start.x, 2) + pow (line_end.y - line_start.y, 2));

            if (line_length >= 0.25 && line_length <= 0.35) {
                best_inliers       = inliers;
                best_line.a        = a;
                best_line.b        = b;
                best_inlier_points = inlier_points;
            }
        }
    }
    std::sort (best_inlier_points.begin (), best_inlier_points.end (), [] (const Point& a, const Point& b) { return a.x < b.x; });
    best_line.start   = best_inlier_points.front ();
    best_line.end     = best_inlier_points.back ();
    best_line.start.y = best_line.a * best_line.start.x + best_line.b;
    best_line.end.y   = best_line.a * best_line.end.x + best_line.b;
    return best_line;
}

}  // namespace e_box_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (e_box_perception::e_box_perception)
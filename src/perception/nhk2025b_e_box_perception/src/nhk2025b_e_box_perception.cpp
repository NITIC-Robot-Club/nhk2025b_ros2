#include "nhk2025b_e_box_perception.hpp"

namespace e_box_perception {

e_box_perception::e_box_perception (const rclcpp::NodeOptions& options) : rclcpp::Node ("e_box_perception", options) {
    box_publisher_            = this->create_publisher<nhk2025b_msgs::msg::BoxArray> ("/box_state", rclcpp::QoS (10));
    e_collect_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/perception/e_collect_pose", rclcpp::QoS (10));
    e_drop_pose_subscriber_   = this->create_subscription<geometry_msgs::msg::PoseStamped> ("/behavior/e_drop_pose", 1, std::bind (&e_box_perception::pose_callback, this, std::placeholders::_1));
    lidar_subscriber_         = this->create_subscription<sensor_msgs::msg::PointCloud2> ("/sensor/lidar", 1, std::bind (&e_box_perception::lidar_callback, this, std::placeholders::_1));
    is_red_subscriber_        = this->create_subscription<std_msgs::msg::Bool> ("/is_red", 1, std::bind (&e_box_perception::is_red_callback, this, std::placeholders::_1));
    iter                      = this->declare_parameter<int> ("iter", 100);
    distance_threshold        = this->declare_parameter<double> ("distance_threshold", 0.025);
    normal_distance           = this->declare_parameter<double> ("normal_distance", 0.5);
}

void e_box_perception::pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    double ransac_length                                 = 0.3;
    e_drop_pose_                                         = *pose;
    std::vector<e_box_perception::Point> points          = cloud_to_points (lidar_data_);                          // 点群データをPointの配列に変換
    std::vector<e_box_perception::Point> filtered_points = filtering_points (points);                              // 検出範囲に入っている点のみ抽出
    e_box_perception::Line               line1           = ransac (filtered_points, ransac_length);                // RANSACで直線を検出
    e_box_perception::Point              centre_of_line  = line_centre (line1);                                    // 直線の中心点を計算
    e_box_perception::Point              centre_of_box   = normal_point (line1, centre_of_line, normal_distance);  // 検出線分の中点からボックスの中心の座標を取得
    e_box_perception::Line               line2;
    line2.start                           = centre_of_box;
    line2.end                             = centre_of_line;
    e_box_perception::Point collect_point = collect_normal_point (line2, centre_of_box, 0.45);                 // ボックスの中心から収集位置までの座標を取得
    double                  line_angle    = atan2 (line1.end.y - line1.start.y, line1.end.x - line1.start.x);  // 収集位置の傾きを取得
    tf2::Quaternion         quat;
    quat.setRPY (0, 0, line_angle);

    geometry_msgs::msg::PoseStamped collect_pose;
    collect_pose.header.frame_id    = "base_link";
    collect_pose.header.stamp       = this->now ();
    collect_pose.pose.position.x    = collect_point.x;
    collect_pose.pose.position.y    = collect_point.y;
    collect_pose.pose.position.z    = 0.0;
    collect_pose.pose.orientation.x = quat.x ();
    collect_pose.pose.orientation.y = quat.y ();
    collect_pose.pose.orientation.z = quat.z ();
    collect_pose.pose.orientation.w = quat.w ();
    e_collect_pose_publisher_->publish (collect_pose);

    nhk2025b_msgs::msg::Box box;
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
    lidar_data_ = *lidar;
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
        normal_angle = line_angle - M_PI / 2.0;
    } else {
        normal_angle = line_angle + M_PI / 2.0;
    }
    e_box_perception::Point normal_pt;
    normal_pt.x = point.x + normal_distance * cos (normal_angle);
    normal_pt.y = point.y + normal_distance * sin (normal_angle);
    return normal_pt;
}

e_box_perception::Point e_box_perception::collect_normal_point (e_box_perception::Line line, e_box_perception::Point point, double normal_distance) {
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
    return normal_pt;
}

e_box_perception::Line e_box_perception::ransac (std::vector<e_box_perception::Point> data, double line_length) {
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
    Line                            best_line;  // ax-y+b=0
    std::vector<Point>              best_inlier_points;
    std::random_device              rd;
    std::mt19937                    gen (rd ());
    std::uniform_int_distribution<> dis (0, data.size () - 1);

    double min_length = line_length - 0.05;
    double max_length = line_length + 0.05;

    for (int i = 0; i < iter; i++) {
        Point p1 = data[dis (gen)];
        Point p2 = data[dis (gen)];
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

            double detected_line_length = sqrt (pow (line_end.x - line_start.x, 2) + pow (line_end.y - line_start.y, 2));

            if (detected_line_length >= min_length && detected_line_length <= max_length) {
                best_inliers       = inliers;
                best_line.a        = a;
                best_line.b        = b;
                best_inlier_points = inlier_points;
            }
        }
    }

    if (best_inlier_points.empty ()) {
        Line err_line;
        err_line.a     = 0.0f;
        err_line.b     = 0.0f;
        err_line.start = {0.0f, 0.0f};
        err_line.end   = {0.0f, 0.0f};
        return err_line;
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

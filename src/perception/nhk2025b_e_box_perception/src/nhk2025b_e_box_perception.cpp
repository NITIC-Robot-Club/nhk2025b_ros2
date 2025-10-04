#include "nhk2025b_e_box_perception.hpp"

namespace e_box_perception {

e_box_perception::e_box_perception (const rclcpp::NodeOptions& options) : rclcpp::Node ("e_box_perception", options) {
    box_publisher_            = this->create_publisher<nhk2025b_msgs::msg::BoxArray> ("/box_state", rclcpp::QoS (10));
    e_collect_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/perception/e_collect_pose", rclcpp::QoS (10));
    test_publisher_           = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/perception/test_pose", rclcpp::QoS (10));
    pose_array_publisher_     = this->create_publisher<geometry_msgs::msg::PoseArray> ("/perception/pose_array", rclcpp::QoS (10));
    e_drop_pose_subscriber_   = this->create_subscription<geometry_msgs::msg::PoseStamped> ("/behavior/e_drop_pose", 1, std::bind (&e_box_perception::pose_callback, this, std::placeholders::_1));
    lidar_subscriber_         = this->create_subscription<sensor_msgs::msg::PointCloud2> ("/sensor/lidar", 1, std::bind (&e_box_perception::lidar_callback, this, std::placeholders::_1));
    is_red_subscriber_        = this->create_subscription<std_msgs::msg::Bool> ("/is_red", 1, std::bind (&e_box_perception::is_red_callback, this, std::placeholders::_1));
    current_pose_subscriber_  = this->create_subscription<geometry_msgs::msg::PoseStamped> ("/localization/current_pose", 1, std::bind (&e_box_perception::current_pose_callback, this, std::placeholders::_1));
    iter                      = this->declare_parameter<int> ("iter", 200);
    distance_threshold        = this->declare_parameter<double> ("distance_threshold", 0.025);
    normal_distance           = this->declare_parameter<double> ("normal_distance", 0.5);
    min_x                     = this->declare_parameter<double> ("min_x", 0.5);
    max_x                     = this->declare_parameter<double> ("max_x", 5.0);
    min_y                     = this->declare_parameter<double> ("min_y", 2.1);
    max_y                     = this->declare_parameter<double> ("max_y", 5.0);
    tf_buffer_                = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_              = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);
}

void e_box_perception::pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    detection_count_++;
    std::vector<Point> inliers;
    e_drop_pose_                                         = *pose;
    std::vector<e_box_perception::Point> points          = cloud_to_points (lidar_data_);  // 点群データをPointの配列に変換
    std::vector<e_box_perception::Point> filtered_points = filtering_points (points);      // 検出範囲に入っている点のみ抽出
    if (detection_count_ >= 1) {
        filtered_points = remove_detected_points (filtered_points, detection_areas_);
    }
    e_box_perception::Line best_line = {0, 0, 0};
    best_line                        = ransac_line (filtered_points, inliers);  // RANSACで直線を検出
    detection_areas_.push_back (inliers);
    e_box_perception::Point         centre_of_line = line_midpoint (best_line);                                                                                                        // 直線の中心点を計算
    double                          theta          = atan2 (best_line.end.second - best_line.start.second, best_line.end.first - best_line.start.first);                               // 検出線分の傾きを計算
    e_box_perception::Point         centre_of_box  = {centre_of_line.first - normal_distance * cos (theta - M_PI_2), centre_of_line.second - normal_distance * sin (theta - M_PI_2)};  // ボックスの中心を計算
    geometry_msgs::msg::PoseStamped test;
    test.header.frame_id = "map";
    test.header.stamp    = this->now ();
    test.pose.position.x = centre_of_box.first;
    test.pose.position.y = centre_of_box.second;
    test_publisher_->publish (test);
    tf2::Quaternion q;
    q.setRPY (0, 0, theta);
    e_box_perception::Point collect_point = {centre_of_box.first + 0.45 * cos (theta), centre_of_box.second + 0.45 * sin (theta)};  // 回収地点を計算

    geometry_msgs::msg::PoseStamped collect_pose;
    collect_pose.header.frame_id  = "map";
    collect_pose.header.stamp     = this->now ();
    collect_pose.pose.position.x  = collect_point.first;
    collect_pose.pose.position.y  = collect_point.second;
    collect_pose.pose.position.z  = 0.0;
    collect_pose.pose.orientation = tf2::toMsg (q);
    e_collect_pose_publisher_->publish (collect_pose);

    nhk2025b_msgs::msg::Box box;
    box.info.type       = nhk2025b_msgs::msg::BoxInfo::E;
    box.info.id         = box_array_.boxes.size () + 1;
    box.pose.position.x = centre_of_box.first;
    box.pose.position.y = centre_of_box.second;
    box.pose.position.z = 0.0;
    box.size.x          = 0.3;
    box.size.y          = 1.0;
    box.size.z          = 0.3;
    box_array_.boxes.push_back (box);
}

void e_box_perception::is_red_callback (const std_msgs::msg::Bool::SharedPtr is_red) {
    is_red_ = is_red->data;
    if (is_red_) {
        min_x = 0.5;
        max_x = 5.0;
        min_y = 0.1;
        max_y = 2.1;
    } else {
        min_x = 0.5;
        max_x = 5.0;
        min_y = 2.1;
        max_y = 5.0;
    }
}

void e_box_perception::lidar_callback (const sensor_msgs::msg::PointCloud2::SharedPtr lidar) {
    if (lidar->data.empty ()) return;
    lidar_data_ = *lidar;
}

std::vector<e_box_perception::Point> e_box_perception::cloud_to_points (const sensor_msgs::msg::PointCloud2& cloud) {
    std::vector<e_box_perception::Point> points;
    if (cloud.width == 0 || cloud.height == 0 || cloud.data.empty ()) {
        return points;
    }
    sensor_msgs::PointCloud2ConstIterator<float> iter_x (cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y (cloud, "y");
    geometry_msgs::msg::TransformStamped         tf_map_to_cloud;
    bool                                         need_tf = cloud.header.frame_id != "map";
    if (need_tf) {
        try {
            tf_map_to_cloud = tf_buffer_->lookupTransform ("map", cloud.header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN (this->get_logger (), "TF lookup failed (%s -> map): %s", cloud.header.frame_id.c_str (), ex.what ());
            need_tf = false;  // 変換できなければそのまま使う
        }
    }
    for (; iter_x != iter_x.end (); ++iter_x, ++iter_y) {
        float x = *iter_x;
        float y = *iter_y;
        if (!std::isfinite (x) || !std::isfinite (y)) continue;
        if (need_tf) {
            tf2::Vector3    pt (x, y, 0.0);
            tf2::Quaternion q (tf_map_to_cloud.transform.rotation.x, tf_map_to_cloud.transform.rotation.y, tf_map_to_cloud.transform.rotation.z, tf_map_to_cloud.transform.rotation.w);
            tf2::Vector3    t (tf_map_to_cloud.transform.translation.x, tf_map_to_cloud.transform.translation.y, tf_map_to_cloud.transform.translation.z);
            pt = tf2::quatRotate (q, pt) + t;
            x  = pt.x ();
            y  = pt.y ();
        }
        points.emplace_back (x, y);
    }
    return points;
}

std::vector<e_box_perception::Point> e_box_perception::filtering_points (std::vector<e_box_perception::Point> data) {
    std::vector<e_box_perception::Point> filtered;
    geometry_msgs::msg::PoseArray        test_data;
    test_data.header.frame_id = "map";
    test_data.header.stamp    = this->now ();
    for (const auto& p : data) {
        if (p.first >= min_x && p.first <= max_x && p.second >= min_y && p.second <= max_y) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = p.first;
            pose.position.y = p.second;
            test_data.poses.push_back (pose);
        }
    }
    pose_array_publisher_->publish (test_data);
    for (const auto& p : data) {
        if (p.first >= min_x && p.first <= max_x && p.second >= min_y && p.second <= max_y) {
            filtered.push_back (p);
        }
    }
    return filtered;
}

e_box_perception::Point e_box_perception::find_box_centre (e_box_perception::Line line, e_box_perception::Point point, double normal_distance) {
    if (robot_pose_.header.stamp.sec == 0) {
        RCLCPP_WARN (this->get_logger (), "Robot pose not yet received");
        return point;  // fallback
    }

    double line_angle = atan2 (line.end.second - line.start.second, line.end.first - line.start.first);

    e_box_perception::Point normal_pt1, normal_pt2;
    double                  normal_angle1 = line_angle - M_PI / 2.0;
    double                  normal_angle2 = line_angle + M_PI / 2.0;

    normal_pt1.first  = point.first + normal_distance * cos (normal_angle1);
    normal_pt1.second = point.second + normal_distance * sin (normal_angle1);
    normal_pt2.first  = point.first + normal_distance * cos (normal_angle2);
    normal_pt2.second = point.second + normal_distance * sin (normal_angle2);

    double dist1 = sqrt (pow (normal_pt1.first - robot_pose_.pose.position.x, 2) + pow (normal_pt1.second - robot_pose_.pose.position.y, 2));
    double dist2 = sqrt (pow (normal_pt2.first - robot_pose_.pose.position.x, 2) + pow (normal_pt2.second - robot_pose_.pose.position.y, 2));

    return (dist1 > dist2) ? normal_pt1 : normal_pt2;
}

e_box_perception::Point e_box_perception::find_collect_point (e_box_perception::Line line, e_box_perception::Point point, double normal_distance) {
    double line_angle = atan2 (line.end.second - line.start.second, line.end.first - line.start.first);
    double normal_angle;
    if (is_red_) {
        normal_angle = line_angle + M_PI / 2.0;
    } else {
        normal_angle = line_angle - M_PI / 2.0;
    }
    e_box_perception::Point normal_pt;
    normal_pt.first  = point.first + normal_distance * cos (normal_angle);
    normal_pt.second = point.second + normal_distance * sin (normal_angle);
    return normal_pt;
}

std::tuple<double, double, double> e_box_perception::ransac (const std::vector<Point>& points, std::vector<Point>& inliers_out) {
    if (points.size () < 2) {
        RCLCPP_WARN (this->get_logger (), "Not enough points for RANSAC");
        return {0.0, 0.0, 0.0};
    }
    double                             threshold         = this->get_parameter ("distance_threshold").as_double ();
    int                                max_iter          = this->get_parameter ("iter").as_int ();
    std::tuple<double, double, double> best_line         = {0, 0, 0};  // a, b, c in ax + by + c = 0
    size_t                             best_inlier_count = 0;
    std::random_device                 rd;
    std::mt19937                       gen (rd ());
    std::uniform_int_distribution<>    dis (0, points.size () - 1);
    for (int i = 0; i < max_iter; ++i) {
        auto p1 = points[dis (gen)];
        auto p2 = points[dis (gen)];
        if (p1 == p2) continue;
        double             a = p2.second - p1.second;
        double             b = p1.first - p2.first;
        double             c = p2.first * p1.second - p1.first * p2.second;
        std::vector<Point> inliers;
        for (const auto& pt : points) {
            if (point_line_distance (pt, a, b, c) < threshold) {
                inliers.push_back (pt);
            }
        }
        if (inliers.size () > best_inlier_count) {
            best_inlier_count = inliers.size ();
            best_line         = {a, b, c};
            inliers_out       = inliers;
        }
    }
    return best_line;
}

double e_box_perception::point_line_distance (const Point& pt, double a, double b, double c) {
    return std::fabs (a * pt.first + b * pt.second + c) / std::sqrt (a * a + b * b);
}

void e_box_perception::current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr current_pose) {
    robot_pose_ = *current_pose;
}

e_box_perception::Line e_box_perception::ransac_line (const std::vector<Point>& points, std::vector<Point>& inliers_out) {
    Line line;
    line.a = line.b = line.c = 0;
    line.start = line.end = {0, 0};
    if (points.size () < 2) return line;
    auto [a, b, c] = ransac (points, inliers_out);
    line.a         = a;
    line.b         = b;
    line.c         = c;
    if (inliers_out.empty ()) return line;
    auto minmax_x = std::minmax_element (inliers_out.begin (), inliers_out.end (), [] (const Point& p1, const Point& p2) { return p1.first < p2.first; });
    line.start    = *minmax_x.first;
    line.end      = *minmax_x.second;
    return line;
}

e_box_perception::Point e_box_perception::line_midpoint (e_box_perception::Line line) {
    Point mid;
    mid.first  = (line.start.first + line.end.first) / 2.0;
    mid.second = (line.start.second + line.end.second) / 2.0;
    return mid;
}

std::vector<e_box_perception::Point> e_box_perception::remove_detected_points (const std::vector<Point>& points, const std::vector<std::vector<Point>>& detected_points) {
    std::vector<Point> remaining_points;
    double             tolerance = 0.1;
    for (const auto& pt : points) {
        bool is_detected = false;
        for (const auto& area : detected_points) {
            for (const auto& det_pt : area) {
                double dx = pt.first - det_pt.first;
                double dy = pt.second - det_pt.second;
                if (std::sqrt (dx * dx + dy * dy) < tolerance) {
                    is_detected = true;
                    break;
                }
            }
            if (is_detected) break;
        }
        if (!is_detected) {
            remaining_points.push_back (pt);
        }
    }
    return remaining_points;
}

}  // namespace e_box_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (e_box_perception::e_box_perception)

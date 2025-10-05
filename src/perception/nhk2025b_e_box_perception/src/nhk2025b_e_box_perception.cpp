#include "nhk2025b_e_box_perception.hpp"

namespace e_box_perception {

e_box_perception::e_box_perception (const rclcpp::NodeOptions& options) : Node ("e_box_perception", options) {
    box_publisher_               = this->create_publisher<nhk2025b_msgs::msg::BoxArray> ("/box_state", rclcpp::QoS (10));
    e_collect_pose_publisher_    = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/perception/e_collect_pose", rclcpp::QoS (10));
    test_publisher_              = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/perception/test_pose", rclcpp::QoS (10));
    pose_array_publisher_        = this->create_publisher<geometry_msgs::msg::PoseArray> ("/perception/pose_array", rclcpp::QoS (10));
    detection_area_publisher_    = this->create_publisher<geometry_msgs::msg::PoseArray> ("/perception/detection_area", rclcpp::QoS (10));
    centre_of_detected_line_     = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/perception/centre_of_detected_line", rclcpp::QoS (10));
    e_drop_pose_subscriber_      = this->create_subscription<geometry_msgs::msg::PoseStamped> ("/behavior/e_drop_pose", 1, std::bind (&e_box_perception::pose_callback, this, std::placeholders::_1));
    lidar_subscriber_            = this->create_subscription<sensor_msgs::msg::PointCloud2> ("/sensor/lidar", 1, std::bind (&e_box_perception::lidar_callback, this, std::placeholders::_1));
    is_red_subscriber_           = this->create_subscription<std_msgs::msg::Bool> ("/is_red", 1, std::bind (&e_box_perception::is_red_callback, this, std::placeholders::_1));
    current_pose_subscriber_     = this->create_subscription<geometry_msgs::msg::PoseStamped> ("/localization/current_pose", 1, std::bind (&e_box_perception::current_pose_callback, this, std::placeholders::_1));
    iter                         = this->declare_parameter<int> ("iter", 500);
    distance_threshold           = this->declare_parameter<double> ("distance_threshold", 0.025);
    normal_distance              = this->declare_parameter<double> ("normal_distance", 0.5);
    min_x                        = this->declare_parameter<double> ("min_x", 0.5);
    max_x                        = this->declare_parameter<double> ("max_x", 5.0);
    min_y                        = this->declare_parameter<double> ("min_y", 2.1);
    max_y                        = this->declare_parameter<double> ("max_y", 5.0);
    permissible_segment_distance = this->declare_parameter<double> ("permissible_segment_distance", 0.1);
    tf_buffer_                   = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_                 = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);
}

void e_box_perception::pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    // 手順1 点群フィルタリング
    detection_count_++;
    std::vector<Point> inliers;
    e_drop_pose_ = *pose;
    RCLCPP_INFO (this->get_logger (), "pose degree: %f", atan2 (2.0 * e_drop_pose_.pose.orientation.w * e_drop_pose_.pose.orientation.z, 1.0 - 2.0 * e_drop_pose_.pose.orientation.z * e_drop_pose_.pose.orientation.z) * 180.0 / M_PI);
    std::vector<e_box_perception::Point> points = cloud_to_points (lidar_data_);
    get_robot_forward_area (e_drop_pose_);
    detection_areas_                                     = {(fl), (fr), (br), (bl)};
    std::vector<e_box_perception::Point> filtered_points = filtering_points (points, detection_areas_);

    if (detection_count_ >= 1) {
        filtered_points = remove_detected_points (filtered_points, detected_areas_);
    }

    // 手順2 RANSACで線分検出1
    Line best_line          = {0, 0, 0};
    best_line               = ransac_line (filtered_points, inliers);
    double best_line_length = lineLength (best_line, inliers);
    RCLCPP_INFO (this->get_logger (), "Best line length: %f", best_line_length);
    double theta = atan2 (best_line.end.second - best_line.start.second, best_line.end.first - best_line.start.first);
    RCLCPP_INFO (this->get_logger (), "Detected line angle: %f, %f", theta * 180.0 / M_PI, (theta - M_PI_2) * 180.0 / M_PI);
    detected_areas_.push_back (inliers);
    filtered_points = remove_detected_points (filtered_points, detected_areas_);

    // 手順4 短辺・長辺判定
    bool is_short_line;
    if (fabs (best_line_length - 1.0) < fabs (best_line_length - 0.3)) {
        is_short_line = false;
        RCLCPP_INFO (this->get_logger (), "Long line detected");
    } else {
        is_short_line = true;
        RCLCPP_INFO (this->get_logger (), "Short line detected");
    }

    Point centre_of_line = line_midpoint (best_line);
    // geometry_msgs::msg::PoseStamped col;
    // col.header.frame_id = "map";
    // col.header.stamp    = this->now ();
    // col.pose.position.x = centre_of_line.first;
    // col.pose.position.y = centre_of_line.second;
    // centre_of_detected_line_->publish (col);
    // double theta = atan2 (short_line.end.second - short_line.start.second, short_line.end.first - short_line.start.first);
    // RCLCPP_INFO (this->get_logger (), "Detected line angle: %f, %f", theta * 180.0 / M_PI, (theta - M_PI_2) * 180.0 / M_PI);  // 検出線分の傾きを計算

    // 手順5 辺からbox中心を計算
    Point centre_of_box;
    if (is_short_line) {
        centre_of_box = {centre_of_line.first - normal_distance * cos (theta - M_PI_2), centre_of_line.second - normal_distance * sin (theta - M_PI_2)};  // ボックスの中心を計算
        geometry_msgs::msg::PoseStamped test;
        test.header.frame_id = "map";
        test.header.stamp    = this->now ();
        test.pose.position.x = centre_of_box.first;
        test.pose.position.y = centre_of_box.second;
        test_publisher_->publish (test);
        tf2::Quaternion q;
        q.setRPY (0, 0, theta);
    } else if (!is_short_line) {
        centre_of_box = {centre_of_line.first - 0.15 * cos (theta - M_PI_2), centre_of_line.second - 0.15 * sin (theta - M_PI_2)};
        geometry_msgs::msg::PoseStamped test;
        test.header.frame_id = "map";
        test.header.stamp    = this->now ();
        test.pose.position.x = centre_of_box.first;
        test.pose.position.y = centre_of_box.second;
        test_publisher_->publish (test);
        tf2::Quaternion q;
        q.setRPY (0, 0, theta);
    }

    // 手順6 回収地点を計算
    Point collect_point;
    if (is_short_line) {
        collect_point = {centre_of_box.first + 0.425 * cos (theta), centre_of_box.second + 0.425 * sin (theta)};  // 回収地点を計算
    } else if (!is_short_line) {
        collect_point = {centre_of_box.first - 0.425 * cos (theta - M_PI_2), centre_of_box.second - 0.425 * sin (theta - M_PI_2)};  // 回収地点を計算
    }

    RCLCPP_INFO (this->get_logger (), "Collect point: (%f, %f)", collect_point.first, collect_point.second);
    tf2::Quaternion q;
    geometry_msgs::msg::PoseStamped collect_pose;
    if (is_short_line){
        q.setRPY (0, 0, theta);
        collect_pose;
        collect_pose.header.frame_id  = "map";
        collect_pose.header.stamp     = this->now ();
        collect_pose.pose.position.x  = collect_point.first;
        collect_pose.pose.position.y  = collect_point.second;
        collect_pose.pose.position.z  = 0.0;
        collect_pose.pose.orientation = tf2::toMsg (q);
    } else {
        q.setRPY (0, 0, theta + M_PI_2);
        collect_pose;
        collect_pose.header.frame_id  = "map";
        collect_pose.header.stamp     = this->now ();
        collect_pose.pose.position.x  = collect_point.first;
        collect_pose.pose.position.y  = collect_point.second;
        collect_pose.pose.position.z  = 0.0;
        collect_pose.pose.orientation = tf2::toMsg (q);
    }
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

std::vector<e_box_perception::Point> e_box_perception::filtering_points (std::vector<e_box_perception::Point> data, const std::vector<Point>& polygon) {
    std::vector<Point>            filtered;
    geometry_msgs::msg::PoseArray test_data;
    test_data.header.frame_id = "map";
    test_data.header.stamp    = this->now ();

    for (const auto& p : data) {
        if (isInsidePolygon (p, polygon)) {
            if (p.first >= min_x && p.first <= max_x && p.second >= min_y && p.second <= max_y) {
                geometry_msgs::msg::Pose pose;
                pose.position.x = p.first;
                pose.position.y = p.second;
                test_data.poses.push_back (pose);
                filtered.push_back (p);
            }
        }
    }
    pose_array_publisher_->publish (test_data);
    RCLCPP_INFO (this->get_logger (), "Filtered points: %zu", filtered.size ());
    return filtered;
}

bool e_box_perception::isInsidePolygon (const Point& pt, const std::vector<Point>& polygon) {
    bool   inside = false;
    size_t n      = polygon.size ();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const Point& pi        = polygon[i];
        const Point& pj        = polygon[j];
        bool         intersect = ((pi.second > pt.second) != (pj.second > pt.second)) && (pt.first < (pj.first - pi.first) * (pt.second - pi.second) / (pj.second - pi.second) + pi.first);
        if (intersect) inside = !inside;
    }
    return inside;
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

e_box_perception::Point e_box_perception::line_midpoint (Line line) {
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

void e_box_perception::get_robot_forward_area (const geometry_msgs::msg::PoseStamped& pose) {
    double short_side = 0.45, long_side = 1.15;
    double cx  = pose.pose.position.x;
    double cy  = pose.pose.position.y;
    double yaw = -tf2::getYaw (pose.pose.orientation);
    double fx = cos (yaw), fy = sin (yaw);
    double lx = -sin (yaw), ly = cos (yaw);
    fl = {cx + fx * long_side + lx * short_side, cy + fy * long_side + ly * short_side};
    fr = {cx + fx * long_side - lx * short_side, cy + fy * long_side - ly * short_side};
    bl = {cx - fx * long_side + lx * short_side, cy - fy * long_side + ly * short_side};
    br = {cx - fx * long_side - lx * short_side, cy - fy * long_side - ly * short_side};
    geometry_msgs::msg::PoseArray detection_area;
    detection_area.header.frame_id = "map";
    detection_area.header.stamp    = this->now ();
    geometry_msgs::msg::Pose p1, p2, p3, p4;
    p1.position.x = fl.first;
    p1.position.y = fl.second;
    p1.position.z = 0.0;
    p2.position.x = fr.first;
    p2.position.y = fr.second;
    p2.position.z = 0.0;
    p3.position.x = bl.first;
    p3.position.y = bl.second;
    p3.position.z = 0.0;
    p4.position.x = br.first;
    p4.position.y = br.second;
    p4.position.z = 0.0;
    detection_area.poses.push_back (p1);
    detection_area.poses.push_back (p2);
    detection_area.poses.push_back (p3);
    detection_area.poses.push_back (p4);
    detection_area_publisher_->publish (detection_area);
}

double e_box_perception::lineLength (const Line& line, const std::vector<Point>& inliers) {
    if (inliers.empty ()) {
        return 0.0;
    }
    auto [min_x, max_x] = std::minmax_element (inliers.begin (), inliers.end (), [] (const Point& p1, const Point& p2) { return p1.first < p2.first; });

    auto [min_y, max_y] = std::minmax_element (inliers.begin (), inliers.end (), [] (const Point& p1, const Point& p2) { return p1.second < p2.second; });

    double dx = max_x->first - min_x->first;
    double dy = max_y->second - min_y->second;
    return std::sqrt (dx * dx + dy * dy);
}

}  // namespace e_box_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (e_box_perception::e_box_perception)

#include "nhk2025b_gate_perception/gate_perception.hpp"

namespace gate_perception {
gate_perception::gate_perception (const rclcpp::NodeOptions &options) : Node ("gate_perception", options) {
    gate_pose_publisher_             = this->create_publisher<geometry_msgs::msg::PoseArray> ("/perception/gate_pose", 1);
    gate_detection_publisher_        = this->create_publisher<geometry_msgs::msg::PoseArray> ("/perception/gate_detection_area", 1);
    side_detection_area_publisher_   = this->create_publisher<geometry_msgs::msg::PoseArray> ("/perception/side_detection_area", 1);
    centre_detection_area_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray> ("/perception/centre_detection_area", 1);
    gate_placement_subscriber_       = this->create_subscription<geometry_msgs::msg::PoseStamped> ("/behavior/gate_placement_pose", 1, std::bind (&gate_perception::pose_callback, this, std::placeholders::_1));
    lidar_subscriber_                = this->create_subscription<sensor_msgs::msg::PointCloud2> ("/sensor/lidar", 1, std::bind (&gate_perception::lidar_callback, this, std::placeholders::_1));
    tf_buffer_                       = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_                     = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);
    iter                             = this->declare_parameter<int> ("iter", 500);
    distance_threshold               = this->declare_parameter<double> ("distance_threshold", 0.025);
    gate_detection_width             = this->declare_parameter<double> ("gate_detection_width", 2.0);
    gate_detection_length            = this->declare_parameter<double> ("gate_detection_length", 0.6);
}

void gate_perception::pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 手順1　点群フィルタリング
    std::vector<Point> inliers;
    gate_placement_pose_                       = *msg;
    std::vector<gate_perception::Point> points = cloud_to_points (point_cloud);
    RCLCPP_INFO (this->get_logger (), "pose degree: %f", get_yaw_2d (gate_placement_pose_) * 180.0 / M_PI);
    gate_detection_area_points   = get_gate_detection_area (gate_placement_pose_, gate_detection_width, gate_detection_length);
    side_detection_area_points   = get_side_detection_area (gate_placement_pose_, gate_detection_width / 2.0, gate_detection_length / 2.0, true);
    centre_detection_area_points = get_side_detection_area (gate_placement_pose_, gate_detection_width / 2.0, gate_detection_length / 2.0, false);
    publish_points_pose_array (gate_detection_area_points);
    publish_points_pose_array (side_detection_area_points);
    publish_points_pose_array (centre_detection_area_points);

    // 手順2 RANSACで直線検出1
    Line best_line1 = {0, 0, 0}, second_line1 = {0, 0, 0};
    best_line1 = ransac_line (side_detection_area_points, inliers);
    detected_areas_.push_back (inliers);
    side_detection_area_points = remove_detected_points (side_detection_area_points, detected_areas_);
    second_line1               = ransac_line (side_detection_area_points, inliers);
    detected_areas_.push_back (inliers);
    side_detection_area_points = remove_detected_points (side_detection_area_points, detected_areas_);

    // 手順3 RANSACで直線検出2
    Line best_line2 = {0, 0, 0}, second_line2 = {0, 0, 0};
    best_line2 = ransac_line (centre_detection_area_points, inliers);
    detected_areas_.push_back (inliers);
    centre_detection_area_points = remove_detected_points (centre_detection_area_points, detected_areas_);
    second_line2                 = ransac_line (centre_detection_area_points, inliers);
    detected_areas_.push_back (inliers);
    centre_detection_area_points = remove_detected_points (centre_detection_area_points, detected_areas_);

    // 手順4 各交点を算出
    Point line_intersection1 = line_intersection (best_line1, second_line1);
    Point line_intersection2 = line_intersection (best_line2, second_line2);

    // 手順5 交点同士の中点をとり、poseを生成
    Point                         midpoint = compute_midpoint ({line_intersection1, line_intersection2});
    geometry_msgs::msg::PoseArray gate_detection_pose_array;
    geometry_msgs::msg::Pose      pose;
    pose.position.x  = midpoint.first;
    pose.position.y  = midpoint.second;
    pose.position.z  = 0.0;
    pose.orientation = gate_placement_pose_.pose.orientation;
    gate_detection_pose_array.poses.push_back (pose);
    gate_pose_publisher_->publish (gate_detection_pose_array);
}

void gate_perception::lidar_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    point_cloud = *msg;
}

double gate_perception::get_yaw_2d (const geometry_msgs::msg::PoseStamped &q) {
    double yaw = atan2 (2.0 * (q.pose.orientation.w * q.pose.orientation.z + q.pose.orientation.x * q.pose.orientation.y), 1.0 - 2.0 * (q.pose.orientation.y * q.pose.orientation.y + q.pose.orientation.z * q.pose.orientation.z));
    return yaw;
}
std::vector<gate_perception::Point> gate_perception::cloud_to_points (const sensor_msgs::msg::PointCloud2 &cloud) {
    std::vector<gate_perception::Point> points;
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
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN (this->get_logger (), "TF lookup failed (%s -> map): %s", cloud.header.frame_id.c_str (), ex.what ());
            need_tf = false;
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

std::vector<gate_perception::Point> gate_perception::get_gate_detection_area (const geometry_msgs::msg::PoseStamped &pose, double width, double length) {
    double hw  = width * 0.5;
    double yaw = get_yaw_2d (pose);
    double fx  = std::cos (yaw);
    double fy  = std::sin (yaw);

    double rx = fy;
    double ry = -fx;

    Point left_near  = {pose.pose.position.x - rx * hw, pose.pose.position.y - ry * hw};
    Point right_near = {pose.pose.position.x + rx * hw, pose.pose.position.y + ry * hw};

    double fx_len = pose.pose.position.x + fx * length;
    double fy_len = pose.pose.position.y + fy * length;

    Point left_far  = {fx_len - rx * hw, fy_len - ry * hw};
    Point right_far = {fx_len + rx * hw, fy_len + ry * hw};

    return {left_near, right_near, right_far, left_far};
}

std::vector<gate_perception::Point> gate_perception::get_side_detection_area (const geometry_msgs::msg::PoseStamped &pose, double width, double length, bool right_side) {
    double                 yaw        = get_yaw_2d (pose);
    double                 fx         = std::cos (yaw);
    double                 fy         = std::sin (yaw);
    double                 rx         = fy;
    double                 ry         = -fx;
    double                 hw         = width * 0.5;
    double                 qw         = width * 0.25;
    double                 nx         = pose.pose.position.x;
    double                 ny         = pose.pose.position.y;
    double                 fx_len     = nx + fx * length;
    double                 fy_len     = ny + fy * length;
    double                 dir        = right_side ? 1.0 : -1.0;
    gate_perception::Point near_inner = {nx + rx * dir * qw, ny + ry * dir * qw};
    gate_perception::Point near_outer = {nx + rx * dir * hw, ny + ry * dir * hw};
    gate_perception::Point far_outer  = {fx_len + rx * dir * hw, fy_len + ry * dir * hw};
    gate_perception::Point far_inner  = {fx_len + rx * dir * qw, fy_len + ry * dir * qw};

    return {near_inner, near_outer, far_outer, far_inner};
}

void gate_perception::publish_points_pose_array (const std::vector<gate_perception::Point> points) {
    geometry_msgs::msg::PoseArray gate_detection_pose_array;
    for (const auto &point : points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp    = this->now ();
        pose.pose.position.x = point.first;
        pose.pose.position.y = point.second;
        pose.pose.position.z = 0.0;
        gate_detection_pose_array.poses.push_back (pose.pose);
    }
    gate_detection_publisher_->publish (gate_detection_pose_array);
}

std::tuple<double, double, double> gate_perception::ransac (const std::vector<Point> &points, std::vector<Point> &inliers_out) {
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
        for (const auto &pt : points) {
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

gate_perception::Line gate_perception::ransac_line (const std::vector<Point> &points, std::vector<Point> &inliers_out) {
    Line line;
    line.a = line.b = line.c = 0;
    line.start = line.end = {0, 0};
    if (points.size () < 2) return line;
    auto [a, b, c] = ransac (points, inliers_out);
    line.a         = a;
    line.b         = b;
    line.c         = c;
    if (inliers_out.empty ()) return line;
    auto minmax_x = std::minmax_element (inliers_out.begin (), inliers_out.end (), [] (const Point &p1, const Point &p2) { return p1.first < p2.first; });
    line.start    = *minmax_x.first;
    line.end      = *minmax_x.second;
    return line;
}

std::vector<gate_perception::Point> gate_perception::remove_detected_points (const std::vector<Point> &points, const std::vector<std::vector<Point>> &detected_points) {
    std::vector<Point> remaining_points;
    double             tolerance = 0.1;
    for (const auto &pt : points) {
        bool is_detected = false;
        for (const auto &area : detected_points) {
            for (const auto &det_pt : area) {
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

gate_perception::Point gate_perception::line_intersection (const Line &l1, const Line &l2) {
    double det = l1.a * l2.b - l2.a * l1.b;
    if (std::fabs (det) < 1e-9) {
        return {};
    }
    double x = (l1.b * l2.c - l2.b * l1.c) / det;
    double y = (l2.a * l1.c - l1.a * l2.c) / det;
    return Point{x, y};
}

double gate_perception::point_line_distance (const Point &pt, double a, double b, double c) {
    return std::fabs (a * pt.first + b * pt.second + c) / std::sqrt (a * a + b * b);
}

gate_perception::Point gate_perception::compute_midpoint (const std::vector<Point> &intersections) {
    double mx = 0.0, my = 0.0;
    for (const auto &pt : intersections) {
        mx += pt.first;
        my += pt.second;
    }
    mx /= intersections.size ();
    my /= intersections.size ();
    return Point{mx, my};
}

}  // namespace gate_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (gate_perception::gate_perception)
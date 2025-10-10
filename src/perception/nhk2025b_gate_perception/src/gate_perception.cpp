#include "nhk2025b_gate_perception/gate_perception.hpp"

#include <algorithm>  // for std::min / std::max
#include <limits>

namespace gate_perception {

gate_perception::gate_perception (const rclcpp::NodeOptions &options) : Node ("gate_perception", options) {
    gate_pose_publisher_        = this->create_publisher<geometry_msgs::msg::PoseArray> ("/perception/gate_pose", 1);
    debug_left_detection_area_  = this->create_publisher<visualization_msgs::msg::Marker> ("/debug/left_detection_area", 1);
    debug_right_detection_area_ = this->create_publisher<visualization_msgs::msg::Marker> ("/debug/right_detection_area", 1);
    gate_placement_subscriber_  = this->create_subscription<geometry_msgs::msg::PoseStamped> ("/behavior/gate_placement_pose", 1, std::bind (&gate_perception::pose_callback, this, std::placeholders::_1));
    lidar_subscriber_           = this->create_subscription<sensor_msgs::msg::PointCloud2> ("/sensor/lidar", 1, std::bind (&gate_perception::lidar_callback, this, std::placeholders::_1));
    is_red_subscriber_          = this->create_subscription<std_msgs::msg::Bool> ("/is_red", 1, std::bind (&gate_perception::is_red_callback, this, std::placeholders::_1));
    tf_buffer_                  = std::make_unique<tf2_ros::Buffer> (this->get_clock ());
    tf_listener_                = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);
    iter                        = this->declare_parameter<int> ("iter", 500);
    distance_threshold          = this->declare_parameter<double> ("distance_threshold", 0.025);
    detection_width             = this->declare_parameter<double> ("detection_width", 2.8);
    detection_length            = this->declare_parameter<double> ("detection_length", 1.0);
}

void gate_perception::pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    detection_count_++;
    RCLCPP_INFO (this->get_logger (), "detection count: %d", detection_count_);
    // 手順1　点群フィルタリング
    gate_placement_pose_                     = *msg;
    std::vector<Point> points                = cloud_to_points (point_cloud);
    std::vector<Point> detection_areas_      = get_robot_backward_area (gate_placement_pose_, detection_width, detection_length);
    std::vector<Point> filtered_points       = filtering_points (points, detection_areas_);
    std::vector<Point> left_detection_areas_ = get_rear_side_detection_area (gate_placement_pose_, detection_width, detection_length, false);
    RCLCPP_INFO (
        this->get_logger (), "left detection area corner: (%lf, %lf), (%lf, %lf), (%lf, %lf), (%lf, %lf)", left_detection_areas_[0].first, left_detection_areas_[0].second, left_detection_areas_[1].first, left_detection_areas_[1].second,
        left_detection_areas_[2].first, left_detection_areas_[2].second, left_detection_areas_[3].first, left_detection_areas_[3].second);
    std::vector<Point> right_detection_areas_ = get_rear_side_detection_area (gate_placement_pose_, detection_width, detection_length, true);
    RCLCPP_INFO (
        this->get_logger (), "right detection area corner: (%lf, %lf), (%lf, %lf), (%lf, %lf), (%lf, %lf)", right_detection_areas_[0].first, right_detection_areas_[0].second, right_detection_areas_[1].first, right_detection_areas_[1].second,
        right_detection_areas_[2].first, right_detection_areas_[2].second, right_detection_areas_[3].first, right_detection_areas_[3].second);
    std::vector<Point> left_points  = filtering_points (filtered_points, left_detection_areas_);
    std::vector<Point> right_points = filtering_points (filtered_points, right_detection_areas_);
    RCLCPP_INFO (this->get_logger (), "filtered points size: %lu", filtered_points.size ());
    visualization_msgs::msg::Marker area;
    area.header.frame_id    = "map";
    area.header.stamp       = this->now ();
    area.type               = visualization_msgs::msg::Marker::LINE_STRIP;
    area.action             = visualization_msgs::msg::Marker::ADD;
    area.scale.x            = 0.005;
    area.color.g            = 1.0;
    area.color.a            = 1.0;
    area.pose.orientation.w = 1.0;
    for (const auto &p : left_detection_areas_) {
        geometry_msgs::msg::Point pt;
        pt.x = p.first;
        pt.y = p.second;
        pt.z = 0.0;
        area.points.push_back (pt);
    }
    geometry_msgs::msg::Point pt;
    pt.x = left_detection_areas_[0].first;
    pt.y = left_detection_areas_[0].second;
    pt.z = 0.0;
    area.points.push_back (pt);
    debug_left_detection_area_->publish (area);
    area.points.clear ();
    for (const auto &p : right_detection_areas_) {
        geometry_msgs::msg::Point pt;
        pt.x = p.first;
        pt.y = p.second;
        pt.z = 0.0;
        area.points.push_back (pt);
    }
    pt.x = right_detection_areas_[0].first;
    pt.y = right_detection_areas_[0].second;
    pt.z = 0.0;
    area.points.push_back (pt);
    debug_right_detection_area_->publish (area);

    // 手順2 RANSACで直線検出1
    std::vector<Point> inliers1, inliers2;
    Point              line_intersection1;
    Line               best_line1 = {0, 0, 0}, second_line1 = {0, 0, 0};
    best_line1 = ransac_line (left_points, inliers1);
    std::vector<Point> remaining1;
    for (const auto &pt : points) {
        if (std::find (inliers1.begin (), inliers1.end (), pt) == inliers1.end ()) {
            remaining1.push_back (pt);
        }
    }
    second_line1 = ransac_line (remaining1, inliers2);
    if (second_line1.a == 0 && second_line1.b == 0 && second_line1.c == 0) {
        if (inliers2.empty ()) {
            line_intersection1 = closest_endpoint_to_robot (gate_placement_pose_, best_line1);
        }
    } else {
        line_intersection1 = line_intersection (best_line1, second_line1);
    }
    RCLCPP_INFO (this->get_logger (), "line intersection1: (%f, %f)", line_intersection1.first, line_intersection1.second);

    // 手順3 RANSACで直線検出2
    std::vector<Point> inliers3, inliers4;
    Point              line_intersection2;
    Line               best_line2 = {0, 0, 0}, second_line2 = {0, 0, 0};
    best_line2 = ransac_line (left_points, inliers3);
    std::vector<Point> remaining2;
    for (const auto &pt : points) {
        if (std::find (inliers1.begin (), inliers1.end (), pt) == inliers1.end ()) {
            remaining2.push_back (pt);
        }
    }
    second_line2 = ransac_line (remaining2, inliers4);
    if (second_line2.a == 0 && second_line2.b == 0 && second_line2.c == 0) {
        if (inliers2.empty ()) {
            line_intersection1 = closest_endpoint_to_robot (gate_placement_pose_, best_line2);
        }
    } else {
        line_intersection1 = line_intersection (best_line2, second_line2);
    }
    RCLCPP_INFO (this->get_logger (), "line intersection1: (%f, %f)", line_intersection1.first, line_intersection1.second);

    // 手順4 各交点を算出
    // Point line_intersection1 = line_intersection (best_line1, second_line1);
    // RCLCPP_INFO (this->get_logger (), "line intersection1: (%f, %f)", line_intersection1.first, line_intersection1.second);
    // Point line_intersection2 = line_intersection (best_line2, second_line2);
    // RCLCPP_INFO (this->get_logger (), "line intersection2: (%f, %f)", line_intersection2.first, line_intersection2.second);

    // 手順5 交点同士の中点をとり、poseを生成
    Point                         midpoint = compute_midpoint ({line_intersection1, line_intersection2});
    geometry_msgs::msg::PoseArray gate_detection_pose_array;
    geometry_msgs::msg::Pose      pose;
    pose.position.x = midpoint.first;
    pose.position.y = midpoint.second;
    pose.position.z = 0.0;
    if (detection_count_ >= 2) {
        double yaw = tf2::getYaw (gate_placement_pose_.pose.orientation) + M_PI;
        yaw        = std::atan2 (std::sin (yaw), std::cos (yaw));
        tf2::Quaternion q;
        q.setRPY (0.0, 0.0, yaw);
        pose.orientation = tf2::toMsg (q);
    } else {
        pose.orientation = gate_placement_pose_.pose.orientation;
    }
    gate_detection_pose_array.poses.push_back (pose);
    gate_pose_publisher_->publish (gate_detection_pose_array);
    RCLCPP_INFO (this->get_logger (), "gate detection pose: (%f, %f)", pose.position.x, pose.position.y);
}

void gate_perception::lidar_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    point_cloud = *msg;
}

void gate_perception::is_red_callback (const std_msgs::msg::Bool::SharedPtr is_red) {
    is_red_ = is_red->data;
    if (is_red_) {
        min_x = 0.5;
        max_x = 5.0;
        min_y = -5.0;
        max_y = 0.0;
    } else {
        min_x = 0.5;
        max_x = 5.0;
        min_y = 0.0;
        max_y = 5.0;
    }
}

bool gate_perception::isInsidePolygon (const Point &pt, const std::vector<Point> &polygon) {
    bool   inside = false;
    size_t n      = polygon.size ();
    if (n < 3) return false;
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const Point &pi        = polygon[i];
        const Point &pj        = polygon[j];
        bool         intersect = ((pi.second > pt.second) != (pj.second > pt.second)) && (pt.first < (pj.first - pi.first) * (pt.second - pi.second) / (pj.second - pi.second) + pi.first);
        if (intersect) inside = !inside;
    }
    return inside;
}

std::vector<gate_perception::Point> gate_perception::filtering_points (const std::vector<gate_perception::Point> &data, const std::vector<Point> &polygon) {
    std::vector<Point> filtered;
    for (const auto &p : data) {
        if (isInsidePolygon (p, polygon)) {
            filtered.push_back (p);
        }
    }
    return filtered;
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
    if (intersections.empty ()) {
        RCLCPP_WARN (this->get_logger (), "compute_midpoint: no intersections provided -> returning (0,0)");
        return Point{0.0, 0.0};
    }
    double mx = 0.0, my = 0.0;
    for (const auto &pt : intersections) {
        mx += pt.first;
        my += pt.second;
    }
    mx /= intersections.size ();
    my /= intersections.size ();
    return Point{mx, my};
}

std::vector<gate_perception::Point> gate_perception::get_robot_backward_area (const geometry_msgs::msg::PoseStamped &pose, double width, double length) {
    double yaw = tf2::getYaw (pose.pose.orientation);
    double bx = std::cos (yaw + M_PI), by = std::sin (yaw + M_PI);
    double sx = std::cos (yaw + M_PI_2), sy = std::sin (yaw + M_PI_2);

    double hl = length * 0.5;
    double hw = width * 0.5;

    double cx = pose.pose.position.x + bx * hl;
    double cy = pose.pose.position.y + by * hl;

    std::vector<gate_perception::Point> c (4);
    c[0] = {cx - hl * bx - hw * sx, cy - hl * by - hw * sy};
    c[1] = {cx + hl * bx - hw * sx, cy + hl * by - hw * sy};
    c[2] = {cx + hl * bx + hw * sx, cy + hl * by + hw * sy};
    c[3] = {cx - hl * bx + hw * sx, cy - hl * by + hw * sy};
    return c;
}

std::vector<gate_perception::Point> gate_perception::get_rear_side_detection_area (const geometry_msgs::msg::PoseStamped &pose, double width, double length, bool right_side) {
    double yaw = tf2::getYaw (pose.pose.orientation);
    double bx = std::cos (yaw + M_PI), by = std::sin (yaw + M_PI);
    double sx = std::cos (yaw + M_PI_2), sy = std::sin (yaw + M_PI_2);

    double hl = length * 0.5;
    double hw = width * 0.25;
    double s  = right_side ? 1.0 : -1.0;

    double cx = pose.pose.position.x + bx * hl + s * hw * sx;
    double cy = pose.pose.position.y + by * hl + s * hw * sy;

    std::vector<gate_perception::Point> c (4);
    c[0] = {cx - hl * bx - hw * sx, cy - hl * by - hw * sy};
    c[1] = {cx + hl * bx - hw * sx, cy + hl * by - hw * sy};
    c[2] = {cx + hl * bx + hw * sx, cy + hl * by + hw * sy};
    c[3] = {cx - hl * bx + hw * sx, cy - hl * by + hw * sy};
    return c;
}

gate_perception::Point gate_perception::closest_endpoint_to_robot (const geometry_msgs::msg::PoseStamped &robot_pose, const Line &line) {
    double rx         = robot_pose.pose.position.x;
    double ry         = robot_pose.pose.position.y;
    double dist_start = std::hypot (line.start.first - rx, line.start.second - ry);
    double dist_end   = std::hypot (line.end.first - rx, line.end.second - ry);
    return (dist_start < dist_end) ? line.start : line.end;
}

}  // namespace gate_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (gate_perception::gate_perception)

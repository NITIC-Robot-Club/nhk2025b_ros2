#include "nhk2025b_pose_initializer/pose_initializer.hpp"

namespace pose_initializer {

pose_initializer::pose_initializer(const rclcpp::NodeOptions & options)
: Node("pose_initializer", options) {
    // パラメータ宣言
    this->declare_parameter<double>("ransac_distance_threshold", 0.05);
    this->declare_parameter<int>("ransac_max_iterations", 100);

    lidar_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/sensor/scan", rclcpp::SensorDataQoS(), std::bind(&pose_initializer::lidar_callback, this, std::placeholders::_1));

    pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/localization/initialpose", 10);
}


double pose_initializer::point_line_distance(const Point& pt, double a, double b, double c) {
    return std::fabs(a * pt.first + b * pt.second + c) / std::sqrt(a * a + b * b);
}

std::optional<pose_initializer::Point> pose_initializer::compute_intersection(
    const std::tuple<float, float, float>& l1,
    const std::tuple<float, float, float>& l2)
{
    auto [a1, b1, c1] = l1;
    auto [a2, b2, c2] = l2;

    float det = a1 * b2 - a2 * b1;
    if (std::fabs(det) < 1e-6) {
        return std::nullopt;  // 平行、交点なし
    }

    float x = (b1 * c2 - b2 * c1) / det;
    float y = (a2 * c1 - a1 * c2) / det;
    return std::make_pair(x, y);
}

std::tuple<float, float, float> pose_initializer::compute_yaw_and_position(
    const std::tuple<float, float, float>& line1,
    const std::tuple<float, float, float>& line2,
    const Point& intersection)
{
    auto is_horizontal = [](const std::tuple<float, float, float>& line) {
        float a = std::get<0>(line);
        float b = std::get<1>(line);
        float angle = std::atan2(-a, b);  // 傾きtanθ = -a/b
        float deg = angle * 180.0 / M_PI;
        return deg > 45.0f;  // 90度に近ければ横向き
    };

    const auto& horizontal = is_horizontal(line1) ? line1 : line2;

    float a = std::get<0>(horizontal);
    float b = std::get<1>(horizontal);
    float yaw = std::atan2(-a, b);  // 横向き線の角度（ラジアン）
    yaw += M_PI;  // 反対向きにする（ロボットの向き）

    // 正規化 [-π, π]
    while (yaw > M_PI) yaw -= 2 * M_PI;
    while (yaw < -M_PI) yaw += 2 * M_PI;

    // 交点の反対がロボットの位置
    float x = -intersection.first;
    float y = -intersection.second;

    return {x, y, yaw};
}


std::tuple<double, double, double> pose_initializer::ransac_line_fit(
    const std::vector<Point>& points,
    std::vector<Point>& inliers_out) 
{
    double threshold = this->get_parameter("ransac_distance_threshold").as_double();
    int max_iter = this->get_parameter("ransac_max_iterations").as_int();

    std::tuple<double, double, double> best_line = {0, 0, 0};  // a, b, c in ax + by + c = 0
    size_t best_inlier_count = 0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size() - 1);

    for (int i = 0; i < max_iter; ++i) {
        auto p1 = points[dis(gen)];
        auto p2 = points[dis(gen)];
        if (p1 == p2) continue;

        double a = p2.second - p1.second;
        double b = p1.first - p2.first;
        double c = p2.first * p1.second - p1.first * p2.second;

        std::vector<Point> inliers;
        for (const auto& pt : points) {
            if (point_line_distance(pt, a, b, c) < threshold) {
                inliers.push_back(pt);
            }
        }

        if (inliers.size() > best_inlier_count) {
            best_inlier_count = inliers.size();
            best_line = {a, b, c};
            inliers_out = inliers;
        }
    }

    return best_line;
}

void pose_initializer::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::vector<Point> points;
    double angle = msg->angle_min;

    for (const auto& r : msg->ranges) {
        if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
            double x = r * std::cos(angle);
            double y = r * std::sin(angle);
            points.emplace_back(x, y);
        }
        angle += msg->angle_increment;
    }

    if (points.size() < 10) return;

    std::vector<Point> inliers1, inliers2;
    auto line1 = ransac_line_fit(points, inliers1);

    // inliers1を除去して残りからもう1本
    std::vector<Point> remaining;
    for (const auto& pt : points) {
        if (std::find(inliers1.begin(), inliers1.end(), pt) == inliers1.end()) {
            remaining.push_back(pt);
        }
    }

    if (remaining.size() < 10) return;
    auto line2 = ransac_line_fit(remaining, inliers2);

    RCLCPP_INFO(this->get_logger(), "Line 1: %.2fx + %.2fy + %.2f = 0", std::get<0>(line1), std::get<1>(line1), std::get<2>(line1));
    RCLCPP_INFO(this->get_logger(), "Line 2: %.2fx + %.2fy + %.2f = 0", std::get<0>(line2), std::get<1>(line2), std::get<2>(line2));

    auto maybe_intersection = compute_intersection(line1, line2);
    if (!maybe_intersection) {
        RCLCPP_WARN(this->get_logger(), "Lines are parallel, no intersection found");
        return;
    }
    auto [x, y, yaw] = compute_yaw_and_position(line1, line2, *maybe_intersection);
    
    RCLCPP_INFO(this->get_logger(), "Estimated position: x=%.2f, y=%.2f, yaw=%.2f deg", x, y, yaw * 180.0 / M_PI);
}

}  // namespace pose_initializer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pose_initializer::pose_initializer)

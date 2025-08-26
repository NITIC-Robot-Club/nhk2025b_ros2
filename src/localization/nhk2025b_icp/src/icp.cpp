#include "nhk2025b_icp/icp.hpp"

namespace nhk2025b_icp {

// --- ユーティリティ関数 ---
std::vector<icp::Point2D> icp::scan_to_points (const sensor_msgs::msg::LaserScan& scan, const geometry_msgs::msg::Pose& pose) const {
    std::vector<Point2D> points;
    double               x0 = pose.position.x, y0 = pose.position.y;
    double               yaw = nhk2025b_utils::get_yaw_2d (pose.orientation);
    for (size_t i = 0; i < scan.ranges.size (); ++i) {
        double r = scan.ranges[i];
        if (!std::isfinite (r) || r < scan.range_min || r >= scan.range_max) continue;
        double angle = scan.angle_min + i * scan.angle_increment + yaw;
        points.push_back ({x0 + r * cos (angle), y0 + r * sin (angle)});
    }
    return points;
}

std::vector<icp::Point2D> icp::map_to_points (const nav_msgs::msg::OccupancyGrid& map) const {
    std::vector<Point2D> points;
    double               res = map.info.resolution;
    double               ox  = map.info.origin.position.x;
    double               oy  = map.info.origin.position.y;
    for (uint32_t y = 0; y < map.info.height; ++y) {
        for (uint32_t x = 0; x < map.info.width; ++x) {
            int idx = y * map.info.width + x;
            if (map.data[idx] > 50) {  // occupied
                points.push_back ({ox + x * res, oy + y * res});
            }
        }
    }
    return points;
}

int icp::find_nearest (const Point2D& p, const std::vector<Point2D>& ref) const {
    double min_dist = std::numeric_limits<double>::max ();
    int    min_idx  = -1;
    for (size_t i = 0; i < ref.size (); ++i) {
        double dx = p.x - ref[i].x, dy = p.y - ref[i].y;
        double d = dx * dx + dy * dy;
        if (d < min_dist) {
            min_dist = d;
            min_idx  = i;
        }
    }
    // 外れ値除去: 0.5m^2(=0.25)以上離れていたら無効
    if (min_dist > 0.25) return -1;
    return min_idx;
}

void icp::icp_step (const std::vector<Point2D>& src, const std::vector<Point2D>& tgt, double& dx, double& dy, double& dtheta) const {
    // src: lidar, tgt: map
    if (src.empty () || tgt.empty ()) {
        dx = dy = dtheta = 0;
        return;
    }
    std::vector<Point2D> src_corr, tgt_corr;
    for (const auto& p : src) {
        int idx = find_nearest (p, tgt);
        if (idx >= 0) {
            src_corr.push_back (p);
            tgt_corr.push_back (tgt[idx]);
        }
    }
    if (src_corr.empty ()) {
        dx = dy = dtheta = 0;
        return;
    }
    // 重心計算
    double mx = 0, my = 0, mxp = 0, myp = 0;
    int    N = src_corr.size ();
    for (int i = 0; i < N; ++i) {
        mx += src_corr[i].x;
        my += src_corr[i].y;
        mxp += tgt_corr[i].x;
        myp += tgt_corr[i].y;
    }
    mx /= N;
    my /= N;
    mxp /= N;
    myp /= N;
    // 回転推定
    double Sxx = 0, Sxy = 0, Syx = 0, Syy = 0;
    for (int i = 0; i < N; ++i) {
        double x = src_corr[i].x - mx, y = src_corr[i].y - my;
        double xp = tgt_corr[i].x - mxp, yp = tgt_corr[i].y - myp;
        Sxx += x * xp;
        Sxy += x * yp;
        Syx += y * xp;
        Syy += y * yp;
    }
    double theta = atan2 (Sxy - Syx, Sxx + Syy);
    dtheta       = theta;
    dx           = mxp - (mx * cos (theta) - my * sin (theta));
    dy           = myp - (mx * sin (theta) + my * cos (theta));
}

void icp::apply_pose (geometry_msgs::msg::Pose& pose, double dx, double dy, double dtheta) const {
    // 平行移動
    pose.position.x += dx;
    pose.position.y += dy;
    // 回転
    double yaw = nhk2025b_utils::get_yaw_2d (pose.orientation);
    yaw += dtheta;
    pose.orientation.z = sin (yaw / 2);
    pose.orientation.w = cos (yaw / 2);
}

// --- コールバック・ノード初期化 ---
icp::icp (const rclcpp::NodeOptions& options) : Node ("icp", options) {
    icp_pose_pub_                    = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/localization/icp_pose", 1);
    lidar_sub_                       = this->create_subscription<sensor_msgs::msg::LaserScan> ("/sensor/scan", 1, std::bind (&icp::lidar_callback, this, std::placeholders::_1));
    map_sub_                         = this->create_subscription<nav_msgs::msg::OccupancyGrid> ("/behavior/map", 1, std::bind (&icp::map_callback, this, std::placeholders::_1));
    current_pose_sub_                = this->create_subscription<geometry_msgs::msg::PoseStamped> ("/localization/current_pose", 1, std::bind (&icp::current_pose_callback, this, std::placeholders::_1));
    current_pose_.pose.orientation.w = 1.0;  // 初期化
}

void icp::current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
}

void icp::map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_map_ = *msg;
}

void icp::lidar_callback (const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (latest_map_.data.empty ()) return;
    // 1. scan→点群
    auto scan_points = scan_to_points (*msg, current_pose_.pose);
    // 2. map→点群
    auto map_points = map_to_points (latest_map_);
    if (scan_points.empty () || map_points.empty ()) return;
    // 3. ICP反復
    geometry_msgs::msg::Pose pose_est = current_pose_.pose;
    for (int iter = 0; iter < 20; ++iter) {
        auto   scan_points_est = scan_to_points (*msg, pose_est);
        double dx, dy, dtheta;
        icp_step (scan_points_est, map_points, dx, dy, dtheta);
        // if (std::abs (dx) < 1e-3 && std::abs (dy) < 1e-3 && std::abs (dtheta) < 1e-4) {
        //     RCLCPP_INFO(this->get_logger(), "ICP converged in %d iterations", iter + 1);
        //     break;
        // }
        apply_pose (pose_est, dx, dy, dtheta);
    }
    // 4. publish
    geometry_msgs::msg::PoseStamped out;
    out.header          = msg->header;
    out.header.frame_id = "map";
    out.pose            = pose_est;
    icp_pose_pub_->publish (out);
}

}  // namespace nhk2025b_icp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (nhk2025b_icp::icp)

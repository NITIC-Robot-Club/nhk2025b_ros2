#ifndef __icp_hpp__
#define __icp_hpp__

#include <nhk2025b_utils/get_yaw_2d.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace nhk2025b_icp {
class icp : public rclcpp::Node {
   public:
    icp (const rclcpp::NodeOptions& options);

   private:
    struct Point2D {
        double x, y;
    };
    void lidar_callback (const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void current_pose_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // --- ICP用ユーティリティ関数 ---
    std::vector<Point2D> scan_to_points (const sensor_msgs::msg::LaserScan& scan, const geometry_msgs::msg::Pose& pose) const;
    std::vector<Point2D> map_to_points (const nav_msgs::msg::OccupancyGrid& map) const;
    int                  find_nearest (const Point2D& p, const std::vector<Point2D>& ref) const;
    void                 icp_step (const std::vector<Point2D>& src, const std::vector<Point2D>& tgt, double& dx, double& dy, double& dtheta) const;
    void                 apply_pose (geometry_msgs::msg::Pose& pose, double dx, double dy, double dtheta) const;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    icp_pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr     lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
    geometry_msgs::msg::PoseStamped                                  current_pose_;
    nav_msgs::msg::OccupancyGrid                                     latest_map_;
};
}  // namespace nhk2025b_icp

#endif  //__icp_hpp__

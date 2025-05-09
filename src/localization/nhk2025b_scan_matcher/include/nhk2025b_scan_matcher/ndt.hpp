#ifndef __ndt_hpp__
#define __ndt_hpp__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ndt {

class ndt : public rclcpp::Node {
   public:
    explicit ndt (const rclcpp::NodeOptions& options);

   private:
    // サブスクライバ
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr     scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    // パブリッシャ
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   pointcloud_pub_;

    // TF
    std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::PoseStamped                current_ekf_pose_;
    geometry_msgs::msg::PoseStamped                last_ekf_pose_;
    geometry_msgs::msg::PoseStamped                last_pose_;

    // PCL / NDT
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;

    // コールバック関数
    void scanCallback (const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void mapCallback (const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
    void poseCallback (const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);

    // 内部処理関数
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertMapToPointCloud (const nav_msgs::msg::OccupancyGrid& map);
};

}  // namespace ndt

#endif  // __ndt_hpp__

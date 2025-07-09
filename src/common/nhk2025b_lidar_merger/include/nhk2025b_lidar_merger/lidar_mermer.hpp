#ifndef __lidar_merger__
#define __lidar_merger__

#include <nhk2025b_utils/get_yaw_2d.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace lidar_merger {

class lidar_merger : public rclcpp::Node {
   public:
    explicit lidar_merger (const rclcpp::NodeOptions &options);

   private:
    void                                                         publish_merged_point_cloud2 ();
    sensor_msgs::msg::LaserScan                                  scan1, scan2;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  pointcloud2_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan1_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan2_sub;
    std::shared_ptr<tf2_ros::Buffer>                             tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>                  tf_listener_;
};

}  // namespace lidar_merger

#endif
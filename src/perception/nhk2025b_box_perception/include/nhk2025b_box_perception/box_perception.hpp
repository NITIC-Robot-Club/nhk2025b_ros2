#ifndef __box_perception_hpp__
#define __box_perception_hpp__

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <nhk2025b_msgs/msg/box_array.hpp>
#include <nhk2025b_msgs/msg/box.hpp>
#include <nhk2025b_msgs/msg/line.hpp>
#include <array>
#include <vector>
#include <random>
#include <cmath>
#include <limits>
#include <string>

namespace box_perception {
class box_perception : public rclcpp::Node {
   public:
    box_perception (const rclcpp::NodeOptions &options);

    // ICP関連関数
    geometry_msgs::msg::Pose estimate_pose_icp(const std::vector<geometry_msgs::msg::Point32>& cluster, const std::vector<geometry_msgs::msg::Point32>& map_points);
    void icp_step(const std::vector<geometry_msgs::msg::Point32>& src, const std::vector<geometry_msgs::msg::Point32>& tgt, double& dx, double& dy, double& dtheta);
    void apply_pose(geometry_msgs::msg::Pose& pose, double dx, double dy, double dtheta);

   private:
    rclcpp::Publisher<nhk2025b_msgs::msg::BoxArray>::SharedPtr box_state_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Subscription<nhk2025b_msgs::msg::BoxArray>::SharedPtr box_array_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_red_subscriber_;

    int iteration_num, pc2_index;
    double threshold;

    nhk2025b_msgs::msg::BoxArray last_box_state;
    nav_msgs::msg::OccupancyGrid last_map_state;
    float resolution;
    uint32_t map_width;
    uint32_t map_height;
    double map_position_y;
    double map_position_x;
    bool is_red;

    std::vector<std::array<double,2>> points;
    int x_offset;
    int y_offset;
    int point_step;
    int num_points;
    const uint8_t* data_ptr;

    int max_loop;
    int best_p_cnt;
    std::array<double,3> best_param;
    std::vector<int> best_inliers;

    double min_x;
    double max_x;
    double min_y;
    double max_y;
    double dx;
    double dy;
    double yaw;
    double length_cm;

    rclcpp::TimerBase::SharedPtr timer_;

    // functions
    void point_cloud_callback (const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void box_array_callback (const nhk2025b_msgs::msg::BoxArray::SharedPtr msg);
    void map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void is_red_callback (const std_msgs::msg::Bool::SharedPtr msg);
    void ransac(sensor_msgs::msg::PointCloud2 pc2);
    uint64_t rand_range(uint64_t max);
    double abs(double val);
    double get_distance(nhk2025b_msgs::msg::Line line, geometry_msgs::msg::Point32 point);
};
}  // namespace box_perception

#endif  //__box_perception_hpp__
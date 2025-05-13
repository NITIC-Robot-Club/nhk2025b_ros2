#ifndef __mcl_hpp__
#define __mcl_hpp__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <memory>
#include <queue>
#include <random>
#include <string>
#include <vector>

namespace mcl {

struct Particle {
    double x;
    double y;
    double theta;
    double weight;
};

class mcl : public rclcpp::Node {
   public:
    explicit mcl (const rclcpp::NodeOptions &options);

   private:
    // コールバック
    void map_callback (const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
    void scan_callback (const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void pose_callback (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg);
    void ekf_callback (const geometry_msgs::msg::PoseStamped::SharedPtr ekf_msg);
    void timer_callback ();

    // 内部処理
    void initialize_particles_gaussian (const geometry_msgs::msg::Pose &initial_pose);
    void motion_update (const geometry_msgs::msg::Pose &current, const geometry_msgs::msg::Pose &last);
    void sensor_update (const sensor_msgs::msg::LaserScan &scan);
    void resample_particles ();
    void create_distance_map ();
    bool is_converged () const;

    geometry_msgs::msg::Pose estimate_pose () const;

    // ユーティリティ
    bool   is_pose_valid (double x, double y) const;
    double compute_likelihood (const Particle &p, const sensor_msgs::msg::LaserScan &scan) const;
    bool   get_transform (
          const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &time,
          geometry_msgs::msg::TransformStamped &transform_out) const;

    // ノード内変数
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr                   scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr                  map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr               ekf_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr    particles_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr     distance_map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

    std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::vector<Particle>                   particles_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    geometry_msgs::msg::Pose                ekf_current_pose_;
    geometry_msgs::msg::Pose                ekf_last_pose_;
    geometry_msgs::msg::PoseStamped         last_estimated_pose_;

    rclcpp::TimerBase::SharedPtr timer;
    std::default_random_engine   rng_;

    // パラメータ
    int    num_particles_;
    double motion_noise_linear_;
    double motion_noise_angle_;
    double gaussian_stddev_linear_;
    double gaussian_stddev_angle_;
    int    random_particle_map_num_;

    std::vector<double> distance_map_;
};

}  // namespace mcl

#endif  // __mcl_hpp__

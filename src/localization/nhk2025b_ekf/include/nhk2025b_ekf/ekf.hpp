#ifndef __ekf_hpp__
#define __ekf_hpp__

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ekf {

class ekf : public rclcpp::Node {
   public:
    explicit ekf (const rclcpp::NodeOptions& options);

   private:
    // センサコールバック関数
    void imu_callback (const sensor_msgs::msg::Imu::SharedPtr msg);
    void odom_callback (const nav_msgs::msg::Odometry::SharedPtr msg);
    void lidar_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // EKF予測・更新ステップ
    void predict (const nav_msgs::msg::Odometry& odom, const sensor_msgs::msg::Imu& imu, double dt);
    void update (const geometry_msgs::msg::PoseStamped& pose);

    // 状態変数と共分散行列
    Eigen::VectorXd x_;  // 状態ベクトル: [x, y, yaw, vx, vy, wz]
    Eigen::MatrixXd P_;  // 共分散行列

    // 最新センサ情報
    sensor_msgs::msg::Imu           latest_imu_;
    nav_msgs::msg::Odometry         latest_odom_;
    geometry_msgs::msg::PoseStamped latest_lidar_pose_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    fused_pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr           imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr lidar_sub_;

    // 時刻管理
    rclcpp::Time last_time_;

    // TF
    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

}  // namespace ekf

#endif  // __ekf_hpp__

#include "nhk2025b_ekf/ekf.hpp"

namespace ekf {

ekf::ekf (const rclcpp::NodeOptions& options) : Node ("ekf", options), tf_buffer_ (this->get_clock ()), tf_listener_ (tf_buffer_) {
    x_ = Eigen::VectorXd::Zero (6);         // [x, y, yaw, vx, vy, wz]
    P_ = Eigen::MatrixXd::Identity (6, 6);  // 共分散

    fused_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/localization/ekf/pose", 10);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu> ("/sensor/imu", 10, std::bind (&ekf::imu_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry> (
        "/localization/wheel_odometry", 10, std::bind (&ekf::odom_callback, this, std::placeholders::_1));

    last_time_ = this->get_clock ()->now ();
}

void ekf::imu_callback (const sensor_msgs::msg::Imu::SharedPtr msg) {
    latest_imu_ = *msg;

    rclcpp::Time current_time = this->get_clock ()->now ();
    double       dt           = (current_time - last_time_).seconds ();

    predict (latest_odom_, latest_imu_, dt);
    last_time_ = current_time;
}

void ekf::odom_callback (const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odom_ = *msg;

    // debug
    geometry_msgs::msg::PoseStamped odom_pose;
    odom_pose.header.stamp    = this->get_clock ()->now ();
    odom_pose.header.frame_id = "map";
    odom_pose.pose            = msg->pose.pose;
    fused_pose_pub_->publish (odom_pose);
    // debug
}

void ekf::lidar_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    update (*msg);

    geometry_msgs::msg::PoseStamped fused_pose;
    fused_pose.header.stamp    = this->get_clock ()->now ();
    fused_pose.header.frame_id = "map";
    fused_pose.pose.position.x = x_ (0);
    fused_pose.pose.position.y = x_ (1);
    fused_pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY (0, 0, x_ (2));
    fused_pose.pose.orientation = tf2::toMsg (q);
    fused_pose_pub_->publish (fused_pose);
}

void ekf::predict (const nav_msgs::msg::Odometry& odom, const sensor_msgs::msg::Imu& imu, double dt) {
    double vx = odom.twist.twist.linear.x;
    double vy = odom.twist.twist.linear.y;
    double wz = imu.angular_velocity.z;

    double theta = x_ (2);

    // 位置更新（ロボットの進行方向に移動）
    x_ (0) += (vx * cos (theta) - vy * sin (theta)) * dt;
    x_ (1) += (vx * sin (theta) + vy * cos (theta)) * dt;
    x_ (2) += wz * dt;

    // 状態更新（速度と角速度）
    x_ (3) = vx;
    x_ (4) = vy;
    x_ (5) = wz;

    // 共分散の予測
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero (6, 6);
    Q.diagonal () << 0.05, 0.05, 0.01, 0.1, 0.1, 0.01;
    P_ = P_ + Q * dt;
}
// EKF更新ステップ
void ekf::update (const geometry_msgs::msg::PoseStamped& pose_msg) {
    Eigen::VectorXd z (3);
    z << pose_msg.pose.position.x, pose_msg.pose.position.y,
        std::asin (pose_msg.pose.orientation.z) * 2.0;  // yaw

    Eigen::VectorXd h = x_.head (3);  // 状態ベクトルの位置と角度

    // 観測と予測値の誤差（残差）
    Eigen::VectorXd y = z - h;

    // 角度の差を -π〜π に正規化
    y (2) = std::atan2 (std::sin (y (2)), std::cos (y (2)));

    // 観測誤差が大きすぎる場合は更新をスキップ
    // double error_threshold = 0.5;  // 0.5メートル以内、または5度以内に収束しない場合
    // if (y.norm() > error_threshold) {
    //     RCLCPP_WARN(this->get_logger(), "Observation is unreliable. Skipping update.");
    //     return;  // 更新しない
    // }

    // 観測行列H（位置と角度の更新）
    Eigen::MatrixXd H    = Eigen::MatrixXd::Zero (3, 6);
    H.block<3, 3> (0, 0) = Eigen::MatrixXd::Identity (3, 3);

    // 観測ノイズR（小さく設定している場合、信頼度に応じて動的に調整可能）
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity (3, 3) * 0.001;

    // イノベーション分散S
    Eigen::MatrixXd S = H * P_ * H.transpose () + R;

    // カルマンゲインK
    Eigen::MatrixXd K = P_ * H.transpose () * S.inverse ();

    // 状態の更新
    x_ = x_ + K * y;

    // 共分散行列の更新
    P_ = (Eigen::MatrixXd::Identity (6, 6) - K * H) * P_;
}

}  // namespace ekf

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (ekf::ekf)

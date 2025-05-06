#include "nhk2025b_ekf_localizer/ekf_localizer.hpp"

namespace ekf_localizer {

ekf_localizer::ekf_localizer (const rclcpp::NodeOptions& options)
    : Node ("ekf_localizer", options), tf_buffer_ (this->get_clock ()), tf_listener_ (tf_buffer_) {
    // 状態ベクトルと共分散行列の初期化
    x_ = Eigen::VectorXd::Zero (6);         // [x, y, z, vx, vy, vz]
    P_ = Eigen::MatrixXd::Identity (6, 6);  // 共分散行列

    // パブリッシャの初期化
    fused_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/localization/current_pose", 10);

    // サブスクライバの設定
    imu_sub_ =
        this->create_subscription<sensor_msgs::msg::Imu> ("/sensor/imu", 10, std::bind (&ekf_localizer::imu_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry> (
        "/localization/wheel_odom", 10, std::bind (&ekf_localizer::odom_callback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped> (
        "/localization/lidar_pose", 10, std::bind (&ekf_localizer::lidar_callback, this, std::placeholders::_1));

    // 時間管理
    last_time_ = this->get_clock ()->now ();
}

// IMUのコールバック関数
void ekf_localizer::imu_callback (const sensor_msgs::msg::Imu::SharedPtr msg) {
    // IMUデータを保存
    latest_imu_ = *msg;

    // 前回のタイムスタンプから経過時間を計算
    rclcpp::Time current_time = this->get_clock ()->now ();
    double       dt           = (current_time - last_time_).seconds ();

    // EKFの予測ステップを実行
    predict (latest_odom_, dt);

    // タイムスタンプを更新
    last_time_ = current_time;
}

// オドメトリのコールバック関数
void ekf_localizer::odom_callback (const nav_msgs::msg::Odometry::SharedPtr msg) {
    // オドメトリデータを保存
    latest_odom_ = *msg;

    // IMUのタイミングで予測
    rclcpp::Time current_time = this->get_clock ()->now ();
    double       dt           = (current_time - last_time_).seconds ();

    // EKFの予測ステップを実行
    predict (latest_odom_, dt);

    // タイムスタンプを更新
    last_time_ = current_time;
}

// LiDARのコールバック関数
void ekf_localizer::lidar_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // LiDARのPoseを使って観測更新
    latest_lidar_pose_ = *msg;
    update (latest_lidar_pose_);

    // 更新後の位置をPublish
    geometry_msgs::msg::PoseStamped fused_pose;
    fused_pose.header.stamp    = this->get_clock ()->now ();
    fused_pose.header.frame_id = "map";  // 地図座標系に合わせる場合
    fused_pose.pose.position.x = x_ (0);
    fused_pose.pose.position.y = x_ (1);
    fused_pose.pose.position.z = x_ (2);
    tf2::Quaternion quaternion;
    quaternion.setRPY (0, 0, x_ (2));  // Convert z (yaw) to quaternion
    fused_pose.pose.orientation = tf2::toMsg (quaternion);
    fused_pose_pub_->publish (fused_pose);
}

// EKF予測ステップ
void ekf_localizer::predict (const nav_msgs::msg::Odometry& odom, double dt) {
    // 状態ベクトルの予測（運動モデルを使う）
    double vx = odom.twist.twist.linear.x;
    double vy = odom.twist.twist.linear.y;
    double vz = odom.twist.twist.linear.z;

    // 位置と速度の更新（簡単な積分）
    x_ (0) += vx * dt;
    x_ (1) += vy * dt;
    x_ (2) += vz * dt;

    // 速度も更新
    x_ (3) = vx;
    x_ (4) = vy;
    x_ (5) = vz;

    // 共分散行列の更新（必要に応じて予測誤差を加算）
    // ここでは省略（必要に応じて追加）
}

// EKF更新ステップ（観測更新）
void ekf_localizer::update (const geometry_msgs::msg::PoseStamped& pose) {
    // LiDARの観測値（pose）を使って観測更新
    Eigen::VectorXd z (3);
    z (0) = pose.pose.position.x;
    z (1) = pose.pose.position.y;
    z (2) = pose.pose.position.z;

    // 観測モデルの適用
    Eigen::VectorXd y = z - x_.head<3> ();  // 観測と予測値の差

    // 観測行列H（ここでは簡単な位置の差分を使用）
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity (3, 6);

    // 観測ノイズR（仮定：観測誤差は一定）
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity (3, 3) * 0.01;

    // イノベーション分散S
    Eigen::MatrixXd S = H * P_ * H.transpose () + R;

    // カルマンゲインK
    Eigen::MatrixXd K = P_ * H.transpose () * S.inverse ();

    // 状態の更新
    x_ = x_ + K * y;

    // 共分散行列の更新
    P_ = (Eigen::MatrixXd::Identity (6, 6) - K * H) * P_;
}

}  // namespace ekf_localizer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (ekf_localizer::ekf_localizer)
#include "nhk2025b_wheel_odometry/wheel_odometry.hpp"

namespace wheel_odometry {
wheel_odometry::wheel_odometry (const rclcpp::NodeOptions &options) : Node ("wheel_odometry", options) {
    swerve_subscriber = this->create_subscription<nhk2025b_msgs::msg::Swerve> ("/swerve/result", 1, std::bind (&wheel_odometry::swerve_callback, this, std::placeholders::_1));
    odom_publisher    = this->create_publisher<nav_msgs::msg::Odometry> ("/localization/wheel_odometry", 1);
    pose_publisher    = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/localization/wheel_odometry_pose", 1);
    wheel_radius      = this->declare_parameter ("wheel_radius", 0.0325);
    robot_width       = this->declare_parameter ("robot_width", 0.8);
    robot_length      = this->declare_parameter ("robot_length", 0.6);

    publish_rate_hz       = this->declare_parameter<int> ("publish_rate_hz", 10);

    timer = this->create_wall_timer (std::chrono::milliseconds (1000 / publish_rate_hz), std::bind (&wheel_odometry::timer_callback, this));

    current_x = 0.0;
    current_y = 0.0;

    current_z = 0.0;
    sum_x     = 0.0;
    sum_y     = 0.0;
    sum_z     = 0.0;
    count     = 0;
}

void wheel_odometry::swerve_callback (const nhk2025b_msgs::msg::Swerve::SharedPtr msg) {
    double wheel_positions[4][2] = {
        {+robot_length / 2.0, +robot_width / 2.0},
        {-robot_length / 2.0, +robot_width / 2.0},
        {-robot_length / 2.0, -robot_width / 2.0},
        {+robot_length / 2.0, -robot_width / 2.0}
    };

    double ATA[3][3] = {};  // A^T * A
    double ATb[3]    = {};  // A^T * b

    for (int i = 0; i < 4; ++i) {
        double theta = msg->wheel_angle[i];
        double v     = msg->wheel_speed[i] / 60.0f * 2.0f * M_PI * wheel_radius;

        double dir_x = std::cos (theta);
        double dir_y = std::sin (theta);
        double rx    = wheel_positions[i][0];
        double ry    = wheel_positions[i][1];

        double ax[3] = {1.0f, 0.0f, -ry};
        double ay[3] = {0.0f, 1.0f, +rx};

        double bx = v * dir_x;
        double by = v * dir_y;

        // ATA += ax * ax^T + ay * ay^T
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                ATA[r][c] += ax[r] * ax[c] + ay[r] * ay[c];
            }
            ATb[r] += ax[r] * bx + ay[r] * by;
        }
    }

    // 解く: ATA * x = ATb をガウス消去法で解く（サイズ小さいので直接展開）
    double x[3] = {};
    // まず ATA をコピー（簡潔化のため）
    double A[3][4] = {
        {ATA[0][0], ATA[0][1], ATA[0][2], ATb[0]},
        {ATA[1][0], ATA[1][1], ATA[1][2], ATb[1]},
        {ATA[2][0], ATA[2][1], ATA[2][2], ATb[2]}
    };

    // ガウス消去法
    for (int i = 0; i < 3; ++i) {
        // ピボットの正規化
        double pivot = A[i][i];
        for (int j = i; j < 4; ++j) A[i][j] /= pivot;

        // 他の行の消去
        for (int k = 0; k < 3; ++k) {
            if (k == i) continue;
            double factor = A[k][i];
            for (int j = i; j < 4; ++j) {
                A[k][j] -= factor * A[i][j];
            }
        }
    }

    x[0] = A[0][3];
    x[1] = A[1][3];
    x[2] = A[2][3];

    sum_x += x[0];
    sum_y += x[1];
    sum_z += x[2];
    count++;
}

void wheel_odometry::timer_callback () {
    if (count != 0) {
        current_z += sum_z / count / publish_rate_hz;
        double angle    = std::atan2 (sum_y / count, sum_x / count) + current_z;
        double distance = std::hypot (sum_x / count, sum_y / count);
        current_x += distance * std::cos (angle) / publish_rate_hz;
        current_y += distance * std::sin (angle) / publish_rate_hz;
    }

    while (current_z > +M_PI) {
        current_z -= M_PI * 2;
    }
    while (current_z < -M_PI) {
        current_z += M_PI * 2;
    }

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp            = this->now ();
    odom_msg.header.frame_id         = "map";
    odom_msg.child_frame_id          = "base_link";
    odom_msg.pose.pose.position.x    = current_x;
    odom_msg.pose.pose.position.y    = current_y;
    odom_msg.pose.pose.position.z    = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = std::sin (current_z / 2.0);
    odom_msg.pose.pose.orientation.w = std::cos (current_z / 2.0);
    odom_msg.twist.twist.linear.x    = sum_x / count;
    odom_msg.twist.twist.linear.y    = sum_y / count;
    odom_msg.twist.twist.linear.z    = 0.0;
    odom_msg.twist.twist.angular.x   = 0.0;
    odom_msg.twist.twist.angular.y   = 0.0;
    odom_msg.twist.twist.angular.z   = sum_z / count;

    odom_publisher->publish (odom_msg);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp    = this->now ();
    pose_msg.header.frame_id = "map";
    pose_msg.pose            = odom_msg.pose.pose;
    pose_publisher->publish (pose_msg);

    sum_x = 0;
    sum_y = 0;
    sum_z = 0;
    count = 0;
}

}  // namespace wheel_odometry

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (wheel_odometry::wheel_odometry)
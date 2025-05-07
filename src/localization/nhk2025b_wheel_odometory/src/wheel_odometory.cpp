#include "nhk2025b_wheel_odometory/wheel_odometory.hpp"

namespace wheel_odometory {
wheel_odometory::wheel_odometory (const rclcpp::NodeOptions &options) : Node ("wheel_odometory", options) {
    swerve_subscriber = this->create_subscription<nhk2025b_msgs::msg::Swerve> (
        "/swerve_result", 10, std::bind (&wheel_odometory::swerve_callback, this, std::placeholders::_1));
    odom_publisher = this->create_publisher<nav_msgs::msg::Odometry> ("/localization/wheel_odom", 10);
    timer          = this->create_wall_timer (std::chrono::milliseconds (50), std::bind (&wheel_odometory::timer_callback, this));
    this->declare_parameter ("wheel_radius", 0.031);
    this->declare_parameter ("wheel_position", 0.62);
    current_x = 0.0;
    current_y = 0.0;
    current_z = 0.0;
    sum_x     = 0.0;
    sum_y     = 0.0;
    sum_z     = 0.0;
    count     = 0;
}

void wheel_odometory::swerve_callback (const nhk2025b_msgs::msg::Swerve::SharedPtr msg) {
    this->get_parameter ("wheel_position", wheel_position);
    this->get_parameter ("wheel_radius", wheel_radius);

    double wheel_positions[4][2] = {
        {+wheel_position, +wheel_position},
        {-wheel_position, +wheel_position},
        {-wheel_position, -wheel_position},
        {+wheel_position, -wheel_position}
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

void wheel_odometory::timer_callback () {
    if (count != 0) {
        current_z += sum_z / count * 0.05;
        double angle    = std::atan2 (sum_y / count, sum_x / count) + current_z;
        double distance = std::hypot (sum_x / count, sum_y / count);
        current_x += distance * std::cos (angle) * 0.05;
        current_y += distance * std::sin (angle) * 0.05;
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
    odom_msg.twist.twist.linear.x    = sum_x / 0.05;
    odom_msg.twist.twist.linear.y    = sum_y / 0.05;
    odom_msg.twist.twist.linear.z    = 0.0;
    odom_msg.twist.twist.angular.x   = 0.0;
    odom_msg.twist.twist.angular.y   = 0.0;
    odom_msg.twist.twist.angular.z   = sum_z / 0.05;

    odom_publisher->publish (odom_msg);

    sum_x = 0;
    sum_y = 0;
    sum_z = 0;
    count = 0;
}

}  // namespace wheel_odometory

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (wheel_odometory::wheel_odometory)
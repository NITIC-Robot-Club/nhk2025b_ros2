
#include "nhk2025b_simulation/simulation.hpp"

namespace simulation {
simulation::simulation (const rclcpp::NodeOptions& options) : Node ("simulation", options) {
    swerve_publisher_       = this->create_publisher<nhk2025b_msgs::msg::Swerve> ("/swerve_result", 10);
    robot_status_publisher_ = this->create_publisher<nhk2025b_msgs::msg::RobotStatus> ("/robot_status", 10);
    pose_publisher_         = this->create_publisher<geometry_msgs::msg::PoseStamped> ("/simulation/pose", 10);
    swerve_subscriber_      = this->create_subscription<nhk2025b_msgs::msg::Swerve> (
        "/swerve_cmd", 10, std::bind (&simulation::swerve_callback, this, std::placeholders::_1));
    timer_         = this->create_wall_timer (std::chrono::milliseconds (100), std::bind (&simulation::timer_callback, this));
    wheel_position = this->declare_parameter<double> ("wheel_position", 0.62);
    wheel_radius   = this->declare_parameter<double> ("wheel_radius", 0.031);

    x_     = 0.0f;
    y_     = 0.0f;
    z_     = 0.0f;
    x_sum_ = 0.0f;
    y_sum_ = 0.0f;
    z_sum_ = 0.0f;
    count_ = 0;
    sig_   = true;
}

void simulation::timer_callback () {
    if (count_ != 0 && sig_) {
        z_ += z_sum_ / count_ * 0.1;
        double angle    = std::atan2 (y_sum_ / count_, x_sum_ / count_) + z_;
        if (!std::isnan(angle)) {
            double distance = std::hypot (x_sum_ / count_, y_sum_ / count_);
            x_ += distance * std::cos (angle) * 0.1;
            y_ += distance * std::sin (angle) * 0.1;
        } else {
            RCLCPP_WARN (this->get_logger (), "NaN detected in angle calculation");
            RCLCPP_INFO (this->get_logger (), "x_sum_: %f, y_sum_: %f, z_sum_: %f", x_sum_, y_sum_, z_sum_);
        }
    }

    while (z_ > +M_PI) {
        z_ -= 2.0f * M_PI;
    }
    while (z_ < -M_PI) {
        z_ += 2.0f * M_PI;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp       = this->now ();
    pose.header.frame_id    = "map";
    pose.pose.position.x    = x_;
    pose.pose.position.y    = y_;
    pose.pose.position.z    = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin (z_ / 2.0f);
    pose.pose.orientation.w = std::cos (z_ / 2.0f);
    pose_publisher_->publish (pose);

    nhk2025b_msgs::msg::RobotStatus status;
    status.header.stamp = this->now ();
    status.signal       = sig_;

    x_sum_ = 0.0f;
    y_sum_ = 0.0f;
    z_sum_ = 0.0f;
    count_ = 0;
}

void simulation::swerve_callback (const nhk2025b_msgs::msg::Swerve::SharedPtr msg) {
    swerve_publisher_->publish (*msg);

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
    x_sum_ += x[0];
    y_sum_ += x[1];
    z_sum_ += x[2];
    count_++;
}
}  // namespace simulation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE (simulation::simulation)
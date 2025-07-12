#include "nhk2025b_canable/canable.hpp"

namespace canable {

canable::canable(const rclcpp::NodeOptions &node_options)
    : Node("canable", node_options) {
    this->declare_parameter("retry_open_can", true);
    this->declare_parameter("retry_write_can", true);
    this->declare_parameter("max_retry_write_count", 5);
    this->get_parameter("retry_open_can", retry_open_can);
    this->get_parameter("retry_write_can", retry_write_can);
    this->get_parameter("max_retry_write_count", max_retry_write_count);

    if (init_can_socket() != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN socket");
        return;
    }

    robot_status_pub_ = this->create_publisher<nhk2025b_msgs::msg::RobotStatus>("/robot_status", 1);
    swerve_pub_       = this->create_publisher<nhk2025b_msgs::msg::Swerve>("/swerve/result", 1);
    swerve_sub_       = this->create_subscription<nhk2025b_msgs::msg::Swerve>(
        "/swerve/cmd", 1, std::bind(&canable::swerve_callback, this, std::placeholders::_1));
    conveyor_sub_     = this->create_subscription<nhk2025b_msgs::msg::Conveyor>(
        "/conveyor/cmd", 1, std::bind(&canable::conveyor_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&canable::timer_callback, this));

    std::thread([this]() { this->read_can_socket(); }).detach();
}

canable::~canable() {
    if (can_socket_ >= 0) {
        close(can_socket_);
    }
}

int canable::init_can_socket() {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create CAN socket");
        return -1;
    }

    std::strcpy(ifr_.ifr_name, "can0");
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr_) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get interface index");
        close(can_socket_);
        if (retry_open_can) {
            RCLCPP_INFO(this->get_logger(), "Retrying to open CAN socket...");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            return init_can_socket();
        }
    }

    addr_.can_family  = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    if (bind(can_socket_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to bind CAN socket");
        return -1;
    }

    RCLCPP_INFO(this->get_logger(), "CAN socket initialized and bound to %s", ifr_.ifr_name);
    return 0;
}

void canable::read_can_socket() {
    struct can_frame frame;
    while (rclcpp::ok()) {
        int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
        if (nbytes > 0) {
            switch (frame.can_id) {
                case 0x100:
                    robot_status_.signal       = (frame.data[0] >> 0) & 0x01;
                    robot_status_.is_resetting = (frame.data[0] >> 1) & 0x01;
                    std::memcpy(&robot_status_.voltage, frame.data + 1, sizeof(float));
                    robot_status_pub_->publish(robot_status_);
                    break;

                case 0x110:
                case 0x111:
                case 0x112:
                case 0x113: {
                    int i = frame.can_id - 0x110;
                    std::memcpy(&swerve_cmd_.wheel_angle[i], frame.data, sizeof(float));
                    std::memcpy(&swerve_cmd_.wheel_speed[i], frame.data + 4, sizeof(float));
                    swerve_flag_[i] = true;

                    if (swerve_flag_[0] && swerve_flag_[1] && swerve_flag_[2] && swerve_flag_[3]) {
                        swerve_pub_->publish(swerve_cmd_);
                        for (int j = 0; j < 4; j++) swerve_flag_[j] = false;
                    }
                    break;
                }
            }
        }
    }
}

void canable::swerve_callback(const nhk2025b_msgs::msg::Swerve::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_swerve_ = msg;
}

void canable::conveyor_callback(const nhk2025b_msgs::msg::Conveyor::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_conveyor_ = msg;
}

void canable::timer_callback() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Send swerve data if available
    if (latest_swerve_) {
        for (int i = 0; i < 4; i++) {
            struct can_frame frame;
            std::memset(&frame, 0, sizeof(struct can_frame));
            frame.can_id  = 0x010 + i;
            frame.can_dlc = 8;
            std::memcpy(frame.data, &latest_swerve_->wheel_angle[i], sizeof(float));
            std::memcpy(frame.data + 4, &latest_swerve_->wheel_speed[i], sizeof(float));
            write(can_socket_, &frame, sizeof(struct can_frame));
        }
    }

    // Send conveyor data if available
    if (latest_conveyor_) {
        struct can_frame frame;
        std::memset(&frame, 0, sizeof(struct can_frame));
        frame.can_id  = 0x016;
        frame.can_dlc = 8;
        std::memcpy(frame.data, &latest_conveyor_->conveyor_rpm_right, sizeof(float));
        std::memcpy(frame.data + 4, &latest_conveyor_->conveyor_rpm_left, sizeof(float));
        write(can_socket_, &frame, sizeof(struct can_frame));
    }
}

}  // namespace canable

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(canable::canable)

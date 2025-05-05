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

        if(init_can_socket() != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN socket");
            return;
        }

        // Create publishers and subscribers
        robot_status_pub_ = this->create_publisher<nhk2025b_msgs::msg::RobotStatus>("/robot_status", 10);
        swerve_pub_ = this->create_publisher<nhk2025b_msgs::msg::Swerve>("/swerve_result", 10);
        swerve_sub_ = this->create_subscription<nhk2025b_msgs::msg::Swerve>(
            "/swerve_cmd", 10, std::bind(&canable::swerve_callback, this, std::placeholders::_1));

        // Start CAN read loop
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

        std::strcpy(ifr_.ifr_name, "can0"); // Use CAN interface "can0"
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr_) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get interface index");
            close(can_socket_);
            if(retry_open_can) {
                RCLCPP_INFO(this->get_logger(), "Retrying to open CAN socket...");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                return init_can_socket();
            }
        }

        addr_.can_family = AF_CAN;
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
                    case 0x000: 
                        nhk2025b_msgs::msg::RobotStatus robot_status;
                        robot_status.signal = (frame.data[0] >> 0) & 0x01;      
                        robot_status.is_resetting = (frame.data[0] >> 1) & 0x01;
                        memcpy(&robot_status.battery_voltage, frame.data + 1, sizeof(float));
                        break;
                    case 0x001:
                    case 0x002:
                    case 0x003:
                    case 0x004:
                        memcpy(&swerve_now_.wheel_angle[frame.can_id - 0x001], frame.data, sizeof(float));
                        memcpy(&swerve_now_.wheel_speed[frame.can_id - 0x001], frame.data + 4, sizeof(float));
                        swerve_flag_[frame.can_id - 0x001] = true;
                        if (swerve_flag_[0] && swerve_flag_[1] && swerve_flag_[2] && swerve_flag_[3]) {
                            swerve_pub_->publish(swerve_now_);
                            for(int i = 0; i < 4; i++) {
                                swerve_flag_[i] = false;
                            }
                        }
                        break;
                }
            }
        }
    }

    void canable::swerve_callback(const nhk2025b_msgs::msg::Swerve::SharedPtr msg) {
        for(int i = 0; i < 4; i++) {
            struct can_frame frame;
            std::memset(&frame, 0, sizeof(struct can_frame));
            frame.can_id = 0x101 + i;
            frame.can_dlc = 8;
            std::memcpy(frame.data, &msg->wheel_angle[i], sizeof(float));
            std::memcpy(frame.data + 4, &msg->wheel_speed[i], sizeof(float));
            if (write(can_socket_, &frame, sizeof(struct can_frame)) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to send CAN message");
                if (retry_write_can) {
                    retry_write_count++;
                    if (retry_write_count < max_retry_write_count) {
                        RCLCPP_INFO(this->get_logger(), "Retrying to send CAN message...");
                        write_can_socket(*msg);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Max retry count reached. Giving up.");
                        retry_write_count = 0;
                        init_can_socket();
                    }
                }
            } else {
                retry_write_count = 0;
            }
        }
    }
} // namespace canable

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(canable::canable)
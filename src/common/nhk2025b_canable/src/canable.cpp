#include "nhk2025b_canable/canable.hpp"

namespace canable {

canable::canable (const rclcpp::NodeOptions &node_options) : Node ("canable", node_options) {
    this->declare_parameter ("retry_open_can", true);
    this->declare_parameter ("retry_write_can", true);
    this->declare_parameter ("max_retry_write_count", 5);
    this->get_parameter ("retry_open_can", retry_open_can);
    this->get_parameter ("retry_write_can", retry_write_can);
    this->get_parameter ("max_retry_write_count", max_retry_write_count);

    if (init_can_socket () != 0) {
        RCLCPP_ERROR (this->get_logger (), "Failed to initialize CAN socket");
        return;
    }

    conveyor_sub_ = this->create_subscription<nhk2025b_msgs::msg::Conveyor> (
        "/conveyor/cmd", 1, std::bind (&canable::conveyor_callback, this, std::placeholders::_1));
    pylon_arm_sub_ = this->create_subscription<nhk2025b_msgs::msg::PylonArm> (
        "/pylon_arm/cmd", 1, std::bind (&canable::pylon_arm_callback, this, std::placeholders::_1));
    robot_status_pub_ = this->create_publisher<nhk2025b_msgs::msg::RobotStatus> ("/robot_status", 1);
    swerve_pub_       = this->create_publisher<nhk2025b_msgs::msg::Swerve> ("/swerve/result", 1);
    swerve_sub_ =
        this->create_subscription<nhk2025b_msgs::msg::Swerve> ("/swerve/cmd", 1, std::bind (&canable::swerve_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer (std::chrono::milliseconds (10), std::bind (&canable::timer_callback, this));

    std::thread ([this] () { this->read_can_socket (); }).detach ();
    can_receive_timer_ =
        this->create_wall_timer (std::chrono::milliseconds (100), std::bind (&canable::check_can_receive, this));  // 100msごとにCAN受信確認
}

canable::~canable () {
    if (can_socket_ >= 0) {
        close (can_socket_);
    }
}

int canable::init_can_socket () {
    can_socket_ = socket (PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        RCLCPP_ERROR (this->get_logger (), "Failed to create CAN socket");
        return -1;
    }

    std::strcpy (ifr_.ifr_name, "can0");
    if (ioctl (can_socket_, SIOCGIFINDEX, &ifr_) < 0) {
        RCLCPP_ERROR (this->get_logger (), "Failed to get interface index");
        close (can_socket_);
        if (retry_open_can) {
            RCLCPP_INFO (this->get_logger (), "Retrying to open CAN socket...");
            std::this_thread::sleep_for (std::chrono::milliseconds (500));
            return init_can_socket ();
        }
    }

    addr_.can_family  = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    if (bind (can_socket_, (struct sockaddr *)&addr_, sizeof (addr_)) < 0) {
        RCLCPP_ERROR (this->get_logger (), "Failed to bind CAN socket");
        return -1;
    }

    RCLCPP_INFO (this->get_logger (), "CAN socket initialized and bound to %s", ifr_.ifr_name);
    return 0;
}

void canable::read_can_socket () {
    struct can_frame frame;
    while (rclcpp::ok ()) {
        int nbytes = read (can_socket_, &frame, sizeof (struct can_frame));
        if (nbytes > 0) {
            for (int i = 0; i < 12; i++) {
                if (frame.can_id == id_list[i]) {
                    id_flag[i] = true;  // フラグを立てる
                    break;
                }
            }

            if (frame.can_id == 0x100 && frame.len == 5) {
                power_receive.raw    = frame.data[0];
                robot_status_.signal = power_receive.data.sig;
                float_bytes voltage;
                for (int i = 0; i < 4; i++) {
                    voltage.bytes[i] = frame.data[i + 1];
                }
                robot_status_.voltage[0] = voltage.value;
                if(robot_status_flag_) {
                    robot_status_pub_->publish (robot_status_);
                    robot_status_flag_ = false;
                }
            }

            if(frame.can_id = 0x101 && frame.len == 8) {
                float_bytes voltage[2];
                for (int i = 0; i < 4; i++) {
                    voltage[0].bytes[i] = frame.data[i];
                    voltage[1].bytes[i] = frame.data[i + 4];    
                }
                robot_status_.voltage[1] = voltage[0].value;
                robot_status_.voltage[2] = voltage[1].value;
                robot_status_flag_ = true;
            }

            if (0x111 <= frame.can_id && frame.can_id <= 0x114 && frame.len == 8) {
                int               i = frame.can_id - 0x111;
                union float_bytes swerve_angle, swerve_speed;
                for (int b = 0; b < 4; ++b) {
                    swerve_angle.bytes[b] = frame.data[b];
                    swerve_speed.bytes[b] = frame.data[b + 4];
                }
                swerve_cmd_.wheel_angle[i] = swerve_angle.value;
                swerve_cmd_.wheel_speed[i] = swerve_speed.value;
                swerve_flag_[i]            = true;
                if (swerve_flag_[0] && swerve_flag_[1] && swerve_flag_[2] && swerve_flag_[3]) {
                    swerve_pub_->publish (swerve_cmd_);
                    for (int j = 0; j < 4; j++) swerve_flag_[j] = false;
                }
            }
        }
    }
}
void canable::check_can_receive () {
    std::ostringstream missing_ids;
    for (int i = 0; i < 12; i++) {
        if (!id_flag[i]) {
            if (!missing_ids.str ().empty ()) {
                missing_ids << ", ";
            }
            missing_ids << std::hex << id_list[i];
        }
        id_flag[i] = false;  // フラグをリセット
    }
    if (!missing_ids.str ().empty ()) {
        RCLCPP_WARN (this->get_logger (), "Missing CAN messages: %s", missing_ids.str ().c_str ());
    }
}

void canable::swerve_callback (const nhk2025b_msgs::msg::Swerve::SharedPtr msg) {
    std::lock_guard<std::mutex> lock (data_mutex_);
    latest_swerve_ = msg;
}

void canable::conveyor_callback (const nhk2025b_msgs::msg::Conveyor::SharedPtr msg) {
    std::lock_guard<std::mutex> lock (data_mutex_);
    latest_conveyor_ = msg;
}

void canable::pylon_arm_callback (const nhk2025b_msgs::msg::PylonArm::SharedPtr msg) {
    std::lock_guard<std::mutex> lock (data_mutex_);
    latest_pylon_arm_ = msg;
}

void canable::timer_callback () {
    std::lock_guard<std::mutex> lock (data_mutex_);
    bool                        pylon_expand[2] = {false, false};  // 0: right, 1: left

    if (latest_swerve_) {
        for (int i = 0; i < 4; i++) {
            struct can_frame frame;
            std::memset (&frame, 0, sizeof (struct can_frame));
            frame.can_id  = 0x011 + i;
            frame.can_dlc = 8;
            union float_bytes swerve_angle, swerve_speed;
            swerve_angle.value = latest_swerve_->wheel_angle[i];
            swerve_speed.value = latest_swerve_->wheel_speed[i];
            for (int b = 0; b < 4; ++b) {
                frame.data[b]     = swerve_angle.bytes[b];
                frame.data[b + 4] = swerve_speed.bytes[b];
            }
            if (!write (can_socket_, &frame, sizeof (struct can_frame))) {
                RCLCPP_ERROR (this->get_logger (), "Failed to write to CAN socket");
            }
            std::this_thread::sleep_for (std::chrono::microseconds (500));
        }
    }
    if (latest_pylon_arm_) {
        for (int i = 0; i < 2; i++) {
            pylon_expand[i] = latest_pylon_arm_->expand[i];
            struct can_frame frame;
            std::memset (&frame, 0, sizeof (struct can_frame));
            frame.can_id  = 0x015 + i;
            frame.can_dlc = 8;
            union float_bytes pylon_height, pylon_rpm;
            pylon_height.value = latest_pylon_arm_->height[i];
            pylon_rpm.value    = latest_pylon_arm_->collect_rpm[i];
            for (int b = 0; b < 4; ++b) {
                frame.data[b]     = pylon_height.bytes[b];
                frame.data[b + 4] = pylon_rpm.bytes[b];
            }
            if (!write (can_socket_, &frame, sizeof (struct can_frame))) {
                RCLCPP_ERROR (this->get_logger (), "Failed to write to CAN socket");
            }
            std::this_thread::sleep_for (std::chrono::microseconds (500));
        }
    }

    if (latest_conveyor_) {
        struct can_frame frame;
        std::memset (&frame, 0, sizeof (struct can_frame));
        frame.can_id  = 0x017;
        frame.can_dlc = 8;
        union float_bytes rpm_left, rpm_right;
        rpm_left.value  = latest_conveyor_->conveyor_rpm[0];
        rpm_right.value = latest_conveyor_->conveyor_rpm[1];
        for (int b = 0; b < 4; ++b) {
            frame.data[b]     = rpm_left.bytes[b];
            frame.data[b + 4] = rpm_right.bytes[b];
        }
        if (!write (can_socket_, &frame, sizeof (struct can_frame))) {
            RCLCPP_ERROR (this->get_logger (), "Failed to write to CAN socket");
        }
        std::this_thread::sleep_for (std::chrono::microseconds (500));
    }

    claw_receive.data.expand_pylon_arm_left  = pylon_expand[0];
    claw_receive.data.expand_pylon_arm_right = pylon_expand[1];
    struct can_frame claw_frame;
    std::memset (&claw_frame, 0, sizeof (struct can_frame));
    claw_frame.can_id  = 0x010;
    claw_frame.can_dlc = 1;
    claw_frame.data[0] = claw_receive.raw;
    if (!write (can_socket_, &claw_frame, sizeof (struct can_frame))) {
        RCLCPP_ERROR (this->get_logger (), "Failed to write claw command to CAN socket");
    }
    std::this_thread::sleep_for (std::chrono::microseconds (500));
}

}  // namespace canable

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (canable::canable)

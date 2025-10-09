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
        // return;
    }

    is_red_sub_    = this->create_subscription<std_msgs::msg::Bool> ("/is_red", 1, std::bind (&canable::is_red_callback, this, std::placeholders::_1));
    e_arm_sub_     = this->create_subscription<nhk2025b_msgs::msg::EArm> ("/e_arm/cmd", 1, std::bind (&canable::e_arm_callback, this, std::placeholders::_1));
    swerve_sub_    = this->create_subscription<nhk2025b_msgs::msg::Swerve> ("/swerve/cmd", 1, std::bind (&canable::swerve_callback, this, std::placeholders::_1));
    box_arm_sub_   = this->create_subscription<nhk2025b_msgs::msg::BoxArm> ("/box_arm/cmd", 1, std::bind (&canable::box_arm_callback, this, std::placeholders::_1));
    command_sub_   = this->create_subscription<nhk2025b_msgs::msg::Command> ("/command", 1, std::bind (&canable::command_callback, this, std::placeholders::_1));
    conveyor_sub_  = this->create_subscription<nhk2025b_msgs::msg::Conveyor> ("/conveyor/cmd", 1, std::bind (&canable::conveyor_callback, this, std::placeholders::_1));
    pylon_arm_sub_ = this->create_subscription<nhk2025b_msgs::msg::PylonArm> ("/pylon_arm/cmd", 1, std::bind (&canable::pylon_arm_callback, this, std::placeholders::_1));

    e_arm_controller_sub_     = this->create_subscription<nhk2025b_msgs::msg::EArm> ("/e_arm/controller_cmd", 1, std::bind (&canable::e_arm_controller_callback, this, std::placeholders::_1));
    box_arm_controller_sub_   = this->create_subscription<nhk2025b_msgs::msg::BoxArm> ("/box_arm/controller_cmd", 1, std::bind (&canable::box_arm_controller_callback, this, std::placeholders::_1));
    conveyor_controller_sub_  = this->create_subscription<nhk2025b_msgs::msg::Conveyor> ("/conveyor/controller_cmd", 1, std::bind (&canable::conveyor_controller_callback, this, std::placeholders::_1));
    pylon_arm_controller_sub_ = this->create_subscription<nhk2025b_msgs::msg::PylonArm> ("/pylon_arm/controller_cmd", 1, std::bind (&canable::pylon_arm_controller_callback, this, std::placeholders::_1));

    bno_yaw_pub_         = this->create_publisher<std_msgs::msg::Float32> ("/bno_yaw", 1);
    robomas_current_pub_ = this->create_publisher<std_msgs::msg::Int32> ("/robomas_current", 1);
    e_arm_pub_           = this->create_publisher<nhk2025b_msgs::msg::EArm> ("/e_arm/result", 1);
    swerve_pub_          = this->create_publisher<nhk2025b_msgs::msg::Swerve> ("/swerve/result", 1);
    box_arm_pub_         = this->create_publisher<nhk2025b_msgs::msg::BoxArm> ("/box_arm/result", 1);
    conveyor_pub_        = this->create_publisher<nhk2025b_msgs::msg::Conveyor> ("/conveyor/result", 1);
    pylon_arm_pub_       = this->create_publisher<nhk2025b_msgs::msg::PylonArm> ("/pylon_arm/result", 1);
    missing_can_id_pub_  = this->create_publisher<std_msgs::msg::Int32MultiArray> ("/missing_can_id", 1);
    robot_status_pub_    = this->create_publisher<nhk2025b_msgs::msg::RobotStatus> ("/robot_status", 1);

    timer_ = this->create_wall_timer (std::chrono::milliseconds (10), std::bind (&canable::timer_callback, this));

    std::thread ([this] () { this->read_can_socket (); }).detach ();
    can_receive_timer_ = this->create_wall_timer (std::chrono::milliseconds (100), std::bind (&canable::check_can_receive, this));  // 100msごとにCAN受信確認

    for (int i = 0; i < 2; i++) {
        box_arm_cmd_.height[i]        = 0.0;
        box_arm_cmd_.hand_position[i] = 0.0;
        box_arm_cmd_.expand[i]        = M_PI / 2;
    }

    command_.allow_automate = false;
    command_.signal         = false;
}

canable::~canable () {
    if (can_socket_ >= 0) {
        close (can_socket_);
    }
}

int canable::init_can_socket () {
    if (!can_interface_exists ("can0")) {
        RCLCPP_ERROR (this->get_logger (), "CAN interface 'can0' does not exist");
        can_alive_ = false;
        return -1;
    }
    can_alive_ = true;

    can_socket_ = socket (PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        RCLCPP_ERROR (this->get_logger (), "Failed to create CAN socket");
        return -1;
    }

    std::strcpy (ifr_.ifr_name, "can0");
    if (ioctl (can_socket_, SIOCGIFINDEX, &ifr_) < 0) {
        RCLCPP_ERROR (this->get_logger (), "Failed to get interface index");
        close (can_socket_);
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
            for (int i = 0; i < 17; i++) {
                if (frame.can_id == id_list[i]) {
                    id_flag[i] = true;  // フラグを立てる
                    break;
                }
            }

            if (frame.can_id == 0x100 && frame.len == 5) {
                power_receive.raw    = frame.data[0];
                robot_status_.signal = power_receive.data.sig;
                float_bytes voltage;
                for (int b = 0; b < 4; b++) {
                    voltage.bytes[b] = frame.data[b + 1];
                }
                robot_status_.voltage[0] = voltage.value;
                if (robot_status_flag_) {
                    robot_status_pub_->publish (robot_status_);
                    robot_status_flag_ = false;
                }
                continue;
            }

            if (frame.can_id == 0x101 && frame.len == 8) {
                float_bytes voltage[2];
                for (int b = 0; b < 4; b++) {
                    voltage[0].bytes[b] = frame.data[b];
                    voltage[1].bytes[b] = frame.data[b + 4];
                }

                robot_status_.voltage[1] = voltage[0].value;
                robot_status_.voltage[2] = voltage[1].value;
                robot_status_flag_       = true;
                continue;
            }

            if (0x111 <= frame.can_id && frame.can_id <= 0x114 && frame.len == 8) {
                int               id = frame.can_id - 0x111;
                union float_bytes swerve_angle, swerve_speed;
                for (int b = 0; b < 4; b++) {
                    swerve_angle.bytes[b] = frame.data[b];
                    swerve_speed.bytes[b] = frame.data[b + 4];
                }
                swerve_result_.wheel_angle[id] = swerve_angle.value;
                swerve_result_.wheel_speed[id] = swerve_speed.value;
                swerve_flag_[id]               = true;
                if (swerve_flag_[0] && swerve_flag_[1] && swerve_flag_[2] && swerve_flag_[3]) {
                    swerve_pub_->publish (swerve_result_);
                    for (int i = 0; i < 4; i++) swerve_flag_[i] = false;
                }
                continue;
            }

            if (0x115 <= frame.can_id && frame.can_id <= 0x116 && frame.len == 8) {
                int               id = frame.can_id - 0x115;
                union float_bytes height, expand;
                for (int b = 0; b < 4; b++) {
                    height.bytes[b] = frame.data[b];
                    expand.bytes[b] = frame.data[b + 4];
                }
                pylon_arm_result_.height[id] = height.value;
                pylon_arm_result_.expand[id] = expand.value;
                pylon_arm_pub_->publish (pylon_arm_result_);
                continue;
            }

            if (0x117 <= frame.can_id && frame.can_id <= 0x118 && frame.len == 8) {
                int               id = frame.can_id - 0x117;
                union float_bytes pylon_rpm, conveyor_rpm;
                for (int b = 0; b < 4; b++) {
                    pylon_rpm.bytes[b]    = frame.data[b];
                    conveyor_rpm.bytes[b] = frame.data[b + 4];
                }
                pylon_arm_result_.collect_rpm[id] = pylon_rpm.value;
                conveyor_result_.conveyor_rpm[id] = conveyor_rpm.value;
                conveyor_pub_->publish (conveyor_result_);
                continue;
            }

            if (0x122 <= frame.can_id && frame.can_id <= 0x123 && frame.len == 8) {
                int               id = frame.can_id - 0x122;
                union float_bytes height, expand;
                for (int b = 0; b < 4; b++) {
                    height.bytes[b] = frame.data[b];
                    expand.bytes[b] = frame.data[b + 4];
                }
                box_arm_result_.height[id] = height.value;
                box_arm_result_.expand[id] = expand.value;
                box_arm_pub_->publish (box_arm_result_);
                continue;
            }

            if (0x124 <= frame.can_id && frame.can_id <= 0x125 && frame.len == 4) {
                int               id = frame.can_id - 0x124;
                union float_bytes hand_position;
                union float_bytes e;
                for (int b = 0; b < 4; ++b) {
                    hand_position.bytes[b] = frame.data[b];
                    e.bytes[b]             = frame.data[b + 4];
                }
                box_arm_result_.hand_position[id] = hand_position.value;
                continue;
            }

            if (frame.can_id == 0x110 && frame.len == 8) {
                claw_transmit.raw                   = frame.data[0];
                robot_status_.reset_pylon_height[0] = claw_transmit.data.reset_height_0;
                robot_status_.reset_pylon_height[1] = claw_transmit.data.reset_height_1;
                robot_status_.reset_pylon_expand[0] = claw_transmit.data.reset_expand_0;
                robot_status_.reset_pylon_expand[1] = claw_transmit.data.reset_expand_1;
                union float_bytes bno_yaw;
                for (int b = 0; b < 4; b++) {
                    bno_yaw.bytes[b] = frame.data[b + 1];
                }
                std_msgs::msg::Float32 bno_yaw_msg;
                bno_yaw_msg.data = bno_yaw.value;
                bno_yaw_pub_->publish (bno_yaw_msg);
                UInt24 robomas_current;
                for (int b = 0; b < 3; b++) {
                    robomas_current.set_byte (b, frame.data[b + 5]);
                }
                robomas_current_[0]      = robomas_current.get_value ();
                robomas_current_flag_[0] = true;
            }

            if (frame.can_id == 0x120 && frame.len == 4) {
                wing_transmit.raw                     = frame.data[0];
                robot_status_.reset_box_arm_height[0] = wing_transmit.data.reset_height_0;
                robot_status_.reset_box_arm_height[1] = wing_transmit.data.reset_height_1;
                robot_status_.reset_box_arm_hand[0]   = wing_transmit.data.reset_hand_0;
                robot_status_.reset_box_arm_hand[1]   = wing_transmit.data.reset_hand_1;
                robot_status_.reset_box_arm_expand[0] = wing_transmit.data.reset_expand_0;
                robot_status_.reset_box_arm_expand[1] = wing_transmit.data.reset_expand_1;
                robot_status_.reset_e_arm_expand      = wing_transmit.data.reset_e_arm_expand;
                robot_status_.reset_e_arm_get         = wing_transmit.data.reset_e_arm_get;
                UInt24 robomas_current;
                for (int b = 0; b < 3; b++) {
                    robomas_current.set_byte (b, frame.data[b + 1]);
                }
                robomas_current_[1]      = robomas_current.get_value ();
                robomas_current_flag_[1] = true;
            }

            if (robomas_current_flag_[0] && robomas_current_flag_[1]) {
                robomas_current_flag_[0] = false;
                robomas_current_flag_[1] = false;
                std_msgs::msg::Int32 robomas_current_msg;
                robomas_current_msg.data = robomas_current_[0] + robomas_current_[1];
                robomas_current_pub_->publish (robomas_current_msg);
                continue;
            }
        }
    }
}

void canable::check_can_receive () {
    std::ostringstream             missing_ids;
    std_msgs::msg::Int32MultiArray missing_ids_msg;
    for (int i = 0; i < 17; i++) {
        if (!id_flag[i]) {
            if (!missing_ids.str ().empty ()) {
                missing_ids << ", ";
            }
            missing_ids << std::hex << id_list[i];
            missing_ids_msg.data.push_back (id_list[i]);
        }
        id_flag[i] = false;  // フラグをリセット
    }
    if (!missing_ids.str ().empty ()) {
        RCLCPP_WARN (this->get_logger (), "Missing CAN messages: %s", missing_ids.str ().c_str ());
    }
    missing_can_id_pub_->publish (missing_ids_msg);
    bool can_exists = can_interface_exists ("can0");
    if (can_exists && !can_alive_) {
        RCLCPP_INFO (this->get_logger (), "CAN interface 'can0' is back online.");
        init_can_socket ();
    } else if (!can_exists && can_alive_) {
        RCLCPP_ERROR (this->get_logger (), "CAN interface 'can0' is down.");
        if (can_socket_ >= 0) {
            close (can_socket_);
        }
    }
    can_alive_ = can_exists;
}

void canable::is_red_callback (const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock (data_mutex_);
    is_red_ = msg->data;
}

void canable::e_arm_callback (const nhk2025b_msgs::msg::EArm::SharedPtr msg) {
    if (command_.allow_automate) {
        std::lock_guard<std::mutex> lock (data_mutex_);
        e_arm_cmd_ = *msg;
    }
}

void canable::swerve_callback (const nhk2025b_msgs::msg::Swerve::SharedPtr msg) {
    std::lock_guard<std::mutex> lock (data_mutex_);
    swerve_cmd_ = *msg;
}

void canable::box_arm_callback (const nhk2025b_msgs::msg::BoxArm::SharedPtr msg) {
    if (command_.allow_automate) {
        std::lock_guard<std::mutex> lock (data_mutex_);
        box_arm_cmd_ = *msg;
    }
}

void canable::command_callback (const nhk2025b_msgs::msg::Command::SharedPtr msg) {
    std::lock_guard<std::mutex> lock (data_mutex_);
    command_ = *msg;
}

void canable::conveyor_callback (const nhk2025b_msgs::msg::Conveyor::SharedPtr msg) {
    if (command_.allow_automate) {
        std::lock_guard<std::mutex> lock (data_mutex_);
        conveyor_cmd_ = *msg;
    }
}

void canable::pylon_arm_callback (const nhk2025b_msgs::msg::PylonArm::SharedPtr msg) {
    if (command_.allow_automate) {
        std::lock_guard<std::mutex> lock (data_mutex_);
        pylon_arm_cmd_ = *msg;
    }
}

void canable::e_arm_controller_callback (const nhk2025b_msgs::msg::EArm::SharedPtr msg) {
    if (!command_.allow_automate) {
        std::lock_guard<std::mutex> lock (data_mutex_);
        e_arm_cmd_ = *msg;
    }
}

void canable::box_arm_controller_callback (const nhk2025b_msgs::msg::BoxArm::SharedPtr msg) {
    if (!command_.allow_automate) {
        std::lock_guard<std::mutex> lock (data_mutex_);
        box_arm_cmd_ = *msg;
    }
}

void canable::conveyor_controller_callback (const nhk2025b_msgs::msg::Conveyor::SharedPtr msg) {
    if (!command_.allow_automate) {
        std::lock_guard<std::mutex> lock (data_mutex_);
        conveyor_cmd_ = *msg;
    }
}

void canable::pylon_arm_controller_callback (const nhk2025b_msgs::msg::PylonArm::SharedPtr msg) {
    if (!command_.allow_automate) {
        std::lock_guard<std::mutex> lock (data_mutex_);
        pylon_arm_cmd_ = *msg;
    }
}

void canable::timer_callback () {
    std::lock_guard<std::mutex> lock (data_mutex_);

    claw_receive.data.reset = command_.reset_claw;
    wing_receive.data.reset = command_.reset_wing;

    struct can_frame power;
    std::memset (&power, 0, sizeof (struct can_frame));
    power.can_id              = 0x000;
    power.can_dlc             = 2;
    power_receive.data.sig    = command_.signal;
    power_receive.data.is_red = is_red_;
    power.data[0]             = power_receive.raw;
    power.data[1]             = 128;
    if (!write (can_socket_, &power, sizeof (struct can_frame))) {
        RCLCPP_ERROR (this->get_logger (), "Failed to write to CAN socket");
    }
    std::this_thread::sleep_for (std::chrono::microseconds (500));

    std::array<uint8_t, max_led> leds{};
    leds.fill (0);

    // 正逆方向計算
    int led_index = (led_step_ <= max_led / 2) ? led_step_ : max_led - led_step_;
    if (led_index >= 0 && led_index < max_led) leds[led_index] = 1;

    // --- 0x001: LED0〜63 ---
    struct can_frame led_frame1{};
    led_frame1.can_id  = 0x001;
    led_frame1.can_dlc = 8;
    for (int i = 0; i < 8; ++i) {      // 8バイト
        for (int b = 0; b < 8; ++b) {  // 8ビット
            int idx = i * 8 + b;
            if (idx < 64 && leds[idx]) led_frame1.data[i] |= (1 << b);
        }
    }
    if (write (can_socket_, &led_frame1, sizeof (led_frame1)) <= 0) {
        RCLCPP_ERROR (this->get_logger (), "Failed to write LED frame 0x001");
    }
    std::this_thread::sleep_for (std::chrono::microseconds (500));

    // --- 0x002: LED64〜109 ---
    struct can_frame led_frame2{};
    led_frame2.can_id  = 0x002;
    led_frame2.can_dlc = 8;
    for (int i = 0; i < 8; ++i) {      // 8バイト
        for (int b = 0; b < 8; ++b) {  // 8ビット
            int idx = 64 + i * 8 + b;
            if (idx < max_led && leds[idx]) led_frame2.data[i] |= (1 << b);
        }
    }
    if (write (can_socket_, &led_frame2, sizeof (led_frame2)) <= 0) {
        RCLCPP_ERROR (this->get_logger (), "Failed to write LED frame 0x002");
    }
    std::this_thread::sleep_for (std::chrono::microseconds (500));

    // 次のステップ
    led_step_ = (led_step_ + 1) % (max_led + 1);

    struct can_frame claw_frame;
    std::memset (&claw_frame, 0, sizeof (struct can_frame));
    claw_frame.can_id  = 0x010;
    claw_frame.can_dlc = 1;
    claw_frame.data[0] = claw_receive.raw;
    if (!write (can_socket_, &claw_frame, sizeof (struct can_frame))) {
        RCLCPP_ERROR (this->get_logger (), "Failed to write to CAN socket");
    }
    std::this_thread::sleep_for (std::chrono::microseconds (500));

    struct can_frame wing_frame;
    std::memset (&wing_frame, 0, sizeof (struct can_frame));
    wing_frame.can_id  = 0x020;
    wing_frame.can_dlc = 1;
    wing_frame.data[0] = wing_receive.raw;
    if (!write (can_socket_, &wing_frame, sizeof (struct can_frame))) {
        RCLCPP_ERROR (this->get_logger (), "Failed to write to CAN socket");
    }
    std::this_thread::sleep_for (std::chrono::microseconds (500));

    for (int i = 0; i < 4; i++) {
        struct can_frame swerve;
        std::memset (&swerve, 0, sizeof (struct can_frame));
        swerve.can_id  = 0x011 + i;
        swerve.can_dlc = 8;
        union float_bytes swerve_angle, swerve_speed;
        swerve_angle.value = swerve_cmd_.wheel_angle[i];
        swerve_speed.value = swerve_cmd_.wheel_speed[i];
        for (int b = 0; b < 4; b++) {
            swerve.data[b]     = swerve_angle.bytes[b];
            swerve.data[b + 4] = swerve_speed.bytes[b];
        }
        if (!write (can_socket_, &swerve, sizeof (struct can_frame))) {
            RCLCPP_ERROR (this->get_logger (), "Failed to write to CAN socket");
        }
        std::this_thread::sleep_for (std::chrono::microseconds (500));
    }

    for (int i = 0; i < 2; i++) {
        struct can_frame pylon;
        std::memset (&pylon, 0, sizeof (struct can_frame));
        pylon.can_id  = 0x015 + i;
        pylon.can_dlc = 8;
        union float_bytes pylon_height, pylon_expand;
        pylon_height.value = pylon_arm_cmd_.height[i];
        pylon_expand.value = pylon_arm_cmd_.expand[i];
        for (int b = 0; b < 4; b++) {
            pylon.data[b]     = pylon_height.bytes[b];
            pylon.data[b + 4] = pylon_expand.bytes[b];
        }
        if (!write (can_socket_, &pylon, sizeof (struct can_frame))) {
            RCLCPP_ERROR (this->get_logger (), "Failed to write to CAN socket");
        }
        std::this_thread::sleep_for (std::chrono::microseconds (500));

        struct can_frame pylon_conveyor;
        std::memset (&pylon_conveyor, 0, sizeof (struct can_frame));
        pylon_conveyor.can_id  = 0x017 + i;
        pylon_conveyor.can_dlc = 8;
        union float_bytes pylon_speed, conveyor_speed;
        pylon_speed.value    = pylon_arm_cmd_.collect_rpm[i];
        conveyor_speed.value = conveyor_cmd_.conveyor_rpm[i];
        for (int b = 0; b < 4; b++) {
            pylon_conveyor.data[b]     = pylon_speed.bytes[b];
            pylon_conveyor.data[b + 4] = conveyor_speed.bytes[b];
        }
        if (!write (can_socket_, &pylon_conveyor, sizeof (struct can_frame))) {
            RCLCPP_ERROR (this->get_logger (), "Failed to write to CAN socket");
        }
        std::this_thread::sleep_for (std::chrono::microseconds (500));

        struct can_frame box_arm;
        std::memset (&box_arm, 0, sizeof (struct can_frame));
        box_arm.can_id  = 0x021 + i;
        box_arm.can_dlc = 8;
        union float_bytes box_arm_height, box_arm_expand;
        box_arm_height.value = box_arm_cmd_.height[i];
        box_arm_expand.value = box_arm_cmd_.expand[i];
        for (int b = 0; b < 4; b++) {
            box_arm.data[b]     = box_arm_height.bytes[b];
            box_arm.data[b + 4] = box_arm_expand.bytes[b];
        }
        if (!write (can_socket_, &box_arm, sizeof (struct can_frame))) {
            RCLCPP_ERROR (this->get_logger (), "Failed to write to CAN socket");
        }
        std::this_thread::sleep_for (std::chrono::microseconds (500));

        struct can_frame arm;
        std::memset (&arm, 0, sizeof (struct can_frame));
        arm.can_id  = 0x023 + i;
        arm.can_dlc = 8;
        union float_bytes arm_position_hand;
        union float_bytes arm_position_e;
        arm_position_hand.value = box_arm_cmd_.hand_position[i];
        if(i == 0) {
            arm_position_e.value = e_arm_cmd_.expand;
        } else {
            arm_position_e.value = e_arm_cmd_.get;
        }
        for (int b = 0; b < 4; b++) {
            arm.data[b] = arm_position_hand.bytes[b];
            arm.data[b + 4] = arm_position_e.bytes[b];
        }
        if (!write (can_socket_, &arm, sizeof (struct can_frame))) {
            RCLCPP_ERROR (this->get_logger (), "Failed to write to CAN socket");
        }
        std::this_thread::sleep_for (std::chrono::microseconds (500));
    }

    struct can_frame e_arm;
    std::memset (&e_arm, 0, sizeof (struct can_frame));
    e_arm.can_id  = 0x025;
    e_arm.can_dlc = 8;
    union float_bytes e_arm_expand, e_arm_get;
    e_arm_expand.value = e_arm_cmd_.expand;
    e_arm_get.value    = e_arm_cmd_.get;
    for (int b = 0; b < 4; b++) {
        e_arm.data[b]     = e_arm_expand.bytes[b];
        e_arm.data[b + 4] = e_arm_get.bytes[b];
    }
    if (!write (can_socket_, &e_arm, sizeof (struct can_frame))) {
        RCLCPP_ERROR (this->get_logger (), "Failed to write to CAN socket");
    }
    std::this_thread::sleep_for (std::chrono::microseconds (500));
}

}  // namespace canable

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (canable::canable)

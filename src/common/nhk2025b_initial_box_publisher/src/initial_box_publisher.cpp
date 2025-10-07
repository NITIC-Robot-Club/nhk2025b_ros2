#include "nhk2025b_initial_box_publisher/initial_box_publisher.hpp"

namespace initial_box_publisher {
initial_box_publisher::initial_box_publisher (const rclcpp::NodeOptions &options) : Node ("initial_box_publisher", options) {
    box_array = get_initial_box_array ();
    for (auto box_name : box_names) {
        nhk2025b_msgs::msg::Box box;
        std::string             param_name = box_name.first + std::to_string (box_name.second);
        int                     id         = box_name.second;
        box.info.id                        = id;
        if (box_name.first == "a") {
            box.info.type = nhk2025b_msgs::msg::BoxInfo::A;
            box.size.x    = 0.3;
            box.size.y    = 0.3;
            box.size.z    = 0.3;
        } else if (box_name.first == "b") {
            box.info.type = nhk2025b_msgs::msg::BoxInfo::B;
            box.size.x    = 0.4;
            box.size.y    = 0.4;
            box.size.z    = 0.4;
        } else if (box_name.first == "c") {
            box.info.type = nhk2025b_msgs::msg::BoxInfo::C;
            box.size.x    = 0.5;
            box.size.y    = 0.5;
            box.size.z    = 0.5;
        } else if (box_name.first == "d") {
            box.info.type = nhk2025b_msgs::msg::BoxInfo::D;
            box.size.x    = 0.2;
            box.size.y    = 0.8;
            box.size.z    = 0.2;
        } else if (box_name.first == "e") {
            box.info.type = nhk2025b_msgs::msg::BoxInfo::E;
            box.size.x    = 0.6;
            box.size.y    = 0.2;
            box.size.z    = 0.6;
        }
        this->declare_parameter<double> (param_name + ".position.x", 0.0);
        this->declare_parameter<double> (param_name + ".position.y", 0.0);
        this->declare_parameter<double> (param_name + ".position.z", -1.0);
        this->declare_parameter<double> (param_name + ".orientation.yaw", 0.0);
        box.pose.position.x    = this->get_parameter (param_name + ".position.x").as_double ();
        box.pose.position.y    = this->get_parameter (param_name + ".position.y").as_double ();
        box.pose.position.z    = this->get_parameter (param_name + ".position.z").as_double ();
        double yaw             = this->get_parameter (param_name + ".orientation.yaw").as_double ();
        box.pose.orientation.z = std::sin (yaw / 2.0);
        box.pose.orientation.w = std::cos (yaw / 2.0);
        // Convert yaw to quaternion------------------------------------------------------------------------------------------------------------------

        if (box.pose.position.z < 0.0) continue;
        box_array.boxes.push_back (box);
    }
    box_array_publisher_ = this->create_publisher<nhk2025b_msgs::msg::BoxArray> ("/box_state", 1);
    is_red_subscriber_   = this->create_subscription<std_msgs::msg::Bool> ("/is_red", 1, std::bind (&initial_box_publisher::is_red_callback, this, std::placeholders::_1));
    timer_               = this->create_wall_timer (std::chrono::milliseconds (100), std::bind (&initial_box_publisher::timer_callback, this));

    set_coat_box_array ();
}
void initial_box_publisher::timer_callback () {
    box_array_publisher_->publish (box_array);
}
void initial_box_publisher::is_red_callback (const std_msgs::msg::Bool::SharedPtr msg) {
    if (is_red != msg->data) {
        is_red = msg->data;
        set_coat_box_array ();
    }
}
void initial_box_publisher::set_coat_box_array () {
    for (auto &box : box_array.boxes) {
        box.pose.position.y = std::abs (box.pose.position.y) * (is_red ? -1.0 : 1.0);
    }
}
nhk2025b_msgs::msg::BoxArray initial_box_publisher::get_initial_box_array () {
    nhk2025b_msgs::msg::BoxArray box_array_msg;
    nhk2025b_msgs::msg::Box      box_msg;

    // box_b
    for (int i = 0; i < 3; ++i) {
        nhk2025b_msgs::msg::Box box;
        switch (i) {
            case 0:
                box.pose.position.x = 7.0;
                box.pose.position.y = 5.25;
                box.pose.position.z = 0.14;
                break;
            case 1:
                box.pose.position.x = 7.4;
                box.pose.position.y = 5.25;
                box.pose.position.z = 0.14;
                break;
            case 2:
                box.pose.position.x = 7.8;
                box.pose.position.y = 5.25;
                box.pose.position.z = 0.14;
                break;
        }
        box.size.x    = 0.4;
        box.size.y    = 0.4;
        box.size.z    = 0.4;
        box.info.type = nhk2025b_msgs::msg::BoxInfo::B;
        box.info.id   = 100 + i;
        box_array_msg.boxes.push_back (box);
    }
    // box_c
    for (int i = 0; i < 3; ++i) {
        nhk2025b_msgs::msg::Box box;
        switch (i) {
            case 0:
                box.pose.position.x = 9.75;
                box.pose.position.y = 5.25;
                box.pose.position.z = 0.14;
                break;
            case 1:
                box.pose.position.x = 10.0;
                box.pose.position.y = 5.25;
                box.pose.position.z = 0.64;
                break;
            case 2:
                box.pose.position.x = 10.25;
                box.pose.position.y = 5.25;
                box.pose.position.z = 0.14;
                break;
        }
        box.size.x    = 0.5;
        box.size.y    = 0.5;
        box.size.z    = 0.5;
        box.info.type = nhk2025b_msgs::msg::BoxInfo::C;
        box.info.id   = 100 + i;
        box_array_msg.boxes.push_back (box);
    }
    // box_d
    for (int i = 0; i < 3; ++i) {
        nhk2025b_msgs::msg::Box box;
        switch (i) {
            case 0:
                box.pose.position.x = 8.1;
                box.pose.position.y = 5.25;
                box.pose.position.z = 0.14;
                break;
            case 1:
                box.pose.position.x = 8.3;
                box.pose.position.y = 5.25;
                box.pose.position.z = 0.14;
                break;
            case 2:
                box.pose.position.x = 8.5;
                box.pose.position.y = 5.25;
                box.pose.position.z = 0.14;
                break;
        }
        box.size.x    = 0.2;
        box.size.y    = 0.8;
        box.size.z    = 0.2;
        box.info.type = nhk2025b_msgs::msg::BoxInfo::D;
        box.info.id   = 100 + i;
        box_array_msg.boxes.push_back (box);
    }
    // box_e
    for (int i = 0; i < 3; ++i) {
        nhk2025b_msgs::msg::Box box;
        switch (i) {
            case 0:
                box.pose.position.x = 8.75;
                box.pose.position.y = 5.25;
                box.pose.position.z = 0.14;
                break;
            case 1:
                box.pose.position.x = 9.05;
                box.pose.position.y = 5.25;
                box.pose.position.z = 0.14;
                break;
            case 2:
                box.pose.position.x = 9.35;
                box.pose.position.y = 5.25;
                box.pose.position.z = 0.14;
                break;
        }
        box.size.x    = 0.3;
        box.size.y    = 1.0;
        box.size.z    = 0.3;
        box.info.type = nhk2025b_msgs::msg::BoxInfo::E;
        box.info.id   = 100 + i;
        box_array_msg.boxes.push_back (box);
    }

    return box_array_msg;
}
}  // namespace initial_box_publisher

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (initial_box_publisher::initial_box_publisher)

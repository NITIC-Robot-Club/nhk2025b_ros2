#include "rclcpp/rclcpp.hpp"
#include "nhk2025b_msgs/msg/box_array.hpp"
#include "nhk2025b_msgs/msg/box.hpp"
#include "nhk2025b_msgs/msg/box_info.hpp"

class BoxInitializer : public rclcpp::Node {
public:
    BoxInitializer() : Node("box_initializer") {
        publisher_ = this->create_publisher<nhk2025b_msgs::msg::BoxArray>("/box_state", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&BoxInitializer::publish_boxes, this)
        );

        // box_a
        for (int i = 1; i <= 5; ++i) {
            nhk2025b_msgs::msg::Box box;
            switch (i) {
                case 1: box.pose.position.x = 9.0; box.pose.position.y = 0.8; box.pose.position.z = 0.0; break;
                case 2: box.pose.position.x = 9.0; box.pose.position.y = 2.3; box.pose.position.z = 0.0; break;
                case 3: box.pose.position.x = 9.5; box.pose.position.y = 0.8; box.pose.position.z = 0.0; break;
                case 4: box.pose.position.x = 9.5; box.pose.position.y = 0.8; box.pose.position.z = 0.0; break;
                case 5: box.pose.position.x = 10.3; box.pose.position.y = 2.3; box.pose.position.z = 0.0; break;
                case 6: box.pose.position.x = 10.1; box.pose.position.y = 2.3; box.pose.position.z = 0.0; break;
                case 7: box.pose.position.x = 8.6; box.pose.position.y = 2.3; box.pose.position.z = 0.0; break;
                case 8: box.pose.position.x = 8.8; box.pose.position.y = 2.3; box.pose.position.z = 0.0; break;
                case 9: box.pose.position.x = 9.0; box.pose.position.y = 1.0; box.pose.position.z = 0.0; break;
                case 10: box.pose.position.x = 8.9; box.pose.position.y = 2.3; box.pose.position.z = 0.0; break;
            }
            box.size.x = 0.3;
            box.size.y = 0.3;
            box.size.z = 0.3;
            box.info.type = nhk2025b_msgs::msg::BoxInfo::A;
            boxes_.push_back(box);
        }
        // box_b
        for (int i = 1; i <= 3; ++i) {
            nhk2025b_msgs::msg::Box box;
            switch (i) {
                case 1: box.pose.position.x = 7.0; box.pose.position.y = 5.225; box.pose.position.z = 0.144; break;
                case 2: box.pose.position.x = 7.4; box.pose.position.y = 5.225; box.pose.position.z = 0.144; break;
                case 3: box.pose.position.x = 7.8; box.pose.position.y = 5.225; box.pose.position.z = 0.144; break;
                case 4: box.pose.position.x = 9.0; box.pose.position.y = 1.8; box.pose.position.z = 0.0; break;
                case 5: box.pose.position.x = 9.4; box.pose.position.y = 2.3; box.pose.position.z = 0.0; break;
                case 6: box.pose.position.x = 9.7; box.pose.position.y = 2.3; box.pose.position.z = 0.0; break;
                case 7: box.pose.position.x = 10.0; box.pose.position.y = 1.1; box.pose.position.z = 0.0; break;
                case 8: box.pose.position.x = 10.3; box.pose.position.y = 1.1; box.pose.position.z = 0.0; break;
            }
            box.size.x = 0.4;
            box.size.y = 0.4;
            box.size.z = 0.4;
            box.info.type = nhk2025b_msgs::msg::BoxInfo::B;
            boxes_.push_back(box);
        }
        // box_c
        for (int i = 1; i <= 5; ++i) {
            nhk2025b_msgs::msg::Box box;
            switch (i) {
                case 1: box.pose.position.x = 9.75; box.pose.position.y = 5.225; box.pose.position.z = 0.144; break;
                case 2: box.pose.position.x = 10.0; box.pose.position.y = 5.225; box.pose.position.z = 0.644; break;
                case 3: box.pose.position.x = 10.25; box.pose.position.y = 5.225; box.pose.position.z = 0.144; break;
                case 4: box.pose.position.x = 8.7; box.pose.position.y = 2.0; box.pose.position.z = 0.0; break;
                case 5: box.pose.position.x = 9.5; box.pose.position.y = 1.5; box.pose.position.z = 0.0; break;
            }
            box.size.x = 0.5;
            box.size.y = 0.5;
            box.size.z = 0.5;
            box.info.type = nhk2025b_msgs::msg::BoxInfo::C;
            boxes_.push_back(box);
        }
        // box_d
        for (int i = 1; i <= 3; ++i) {
            nhk2025b_msgs::msg::Box box;
            switch (i) {
                case 1: box.pose.position.x = 8.1; box.pose.position.y = 5.225; box.pose.position.z = 0.144; break;
                case 2: box.pose.position.x = 8.3; box.pose.position.y = 5.225; box.pose.position.z = 0.144; break;
                case 3: box.pose.position.x = 8.5; box.pose.position.y = 5.225; box.pose.position.z = 0.144; break;
            }
            box.size.x = 0.2;
            box.size.y = 0.8;
            box.size.z = 0.2;
            box.info.type = nhk2025b_msgs::msg::BoxInfo::D;
            boxes_.push_back(box);
        }
        // box_e
        for (int i = 1; i <= 3; ++i) {
            nhk2025b_msgs::msg::Box box;
            switch (i) {
                case 1: box.pose.position.x = 8.75; box.pose.position.y = 5.225; box.pose.position.z = 0.144; break;
                case 2: box.pose.position.x = 9.05; box.pose.position.y = 5.225; box.pose.position.z = 0.144; break;
                case 3: box.pose.position.x = 9.35; box.pose.position.y = 5.225; box.pose.position.z = 0.144; break;
            }
            box.size.x = 0.3;
            box.size.y = 1.0;
            box.size.z = 0.3;
            box.info.type = nhk2025b_msgs::msg::BoxInfo::E;
            boxes_.push_back(box);
        }
    }

private:
    void publish_boxes() {
        nhk2025b_msgs::msg::BoxArray msg;
        msg.boxes = boxes_;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<nhk2025b_msgs::msg::BoxArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<nhk2025b_msgs::msg::Box> boxes_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoxInitializer>());
    rclcpp::shutdown();
    return 0;
}
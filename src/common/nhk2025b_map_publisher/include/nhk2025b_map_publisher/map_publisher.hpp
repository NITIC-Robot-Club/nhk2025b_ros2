#ifndef __map_publisher_hpp__
#define __map_publisher_hpp__

#include "nhk2025b_utils/get_yaw_2d.hpp"

#include <algorithm>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nhk2025b_msgs/msg/box_array.hpp>

namespace map_publisher {
class map_publisher : public rclcpp::Node {
   public:
    map_publisher (const rclcpp::NodeOptions& options);

   private:
    nhk2025b_msgs::msg::BoxArray boxes;

    void   box_callback (const nhk2025b_msgs::msg::BoxArray::SharedPtr msg);
    void   publish_map ();
    bool   is_red;
    double resolution_;

    double field_data[5][4] = {
  // {x0,x1,y0,y1}
  // 青ゾーン前提で設定
        {  0.0, 0.15,   0.0,  5.4}, // 手前
        {  0.0, 10.8,   0.0, 0.15}, // 右
        {10.65, 10.8,   0.0,  5.4}, // 奥
        { 6.95, 10.8, 5.175,  5.4}, //  共有
        {  0.0, 10.8,  5.37,  5.4}, // 左
    };

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr    publisher_;
    rclcpp::Subscription<nhk2025b_msgs::msg::BoxArray>::SharedPtr box_subscriber_;
    rclcpp::TimerBase::SharedPtr                                  timer_;
};
}  // namespace map_publisher

#endif  //__map_publisher_hpp__
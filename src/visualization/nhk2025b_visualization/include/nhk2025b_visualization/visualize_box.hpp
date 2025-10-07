#ifndef __nhk2025b_visualize_box_hpp__
#define __nhk2025b_visualize_box_hpp__

#include "rclcpp/rclcpp.hpp"

#include "nhk2025b_msgs/msg/box_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace visualize_box {
class visualize_box : public rclcpp::Node {
   public:
    visualize_box (const rclcpp::NodeOptions &options);

   private:
    void box_callback (const nhk2025b_msgs::msg::BoxArray::SharedPtr msg);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Subscription<nhk2025b_msgs::msg::BoxArray>::SharedPtr      box_subscriber_;

};  // class visualize_box

}  // namespace visualize_box
#endif  // __nhk2025b_visualize_box_hpp__
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "px4_interface/offboard_hover_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_interface::OffboardHoverNode>());
  rclcpp::shutdown();
  return 0;
}

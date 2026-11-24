#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "px4_interface/goal_follower_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_interface::GoalFollowerNode>());
  rclcpp::shutdown();
  return 0;
}

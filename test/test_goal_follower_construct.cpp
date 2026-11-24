#include <memory>

#include "gtest/gtest.h"
#include "px4_interface/goal_follower_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

class GoalFollowerConstructTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

}  // namespace

TEST_F(GoalFollowerConstructTest, StartsInWaitForPx4WithoutPublishingRequirement)
{
  auto node = std::make_shared<px4_interface::GoalFollowerNode>();
  EXPECT_EQ(node->state(), px4_interface::GoalFollowerState::kWaitForPx4);
  // One timer tick without PX4 must remain WAIT_FOR_PX4.
  rclcpp::spin_some(node);
  EXPECT_EQ(node->state(), px4_interface::GoalFollowerState::kWaitForPx4);
}

TEST_F(GoalFollowerConstructTest, RejectsInvalidYawMode)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("yaw_mode", "face_path");
  EXPECT_THROW(
    (void)std::make_shared<px4_interface::GoalFollowerNode>(options),
    std::invalid_argument);
}

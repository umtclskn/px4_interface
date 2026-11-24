#include <chrono>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "nav_msgs/msg/path.hpp"
#include "px4_interface/frame_conversions.hpp"
#include "px4_interface/goal_follower_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

class GoalFollowerPathInputTest : public ::testing::Test
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
    node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  std::shared_ptr<px4_interface::GoalFollowerNode> node_;
};

}  // namespace

TEST_F(GoalFollowerPathInputTest, FrameIdMismatchRejected)
{
  node_ = std::make_shared<px4_interface::GoalFollowerNode>();
  ASSERT_EQ(
    node_->state(), px4_interface::GoalFollowerState::kWaitForPx4);

  const std::vector<px4_interface::Vec3> seed = {
    px4_interface::Vec3{0.0, 0.0, -1.5},
    px4_interface::Vec3{1.0, 0.0, -1.5},
  };
  node_->injectAcceptedPathForTest(seed, 10.0);
  ASSERT_TRUE(node_->hasActivePath());
  ASSERT_EQ(node_->pathRejectCount(), 0u);

  auto path_pub = node_->create_publisher<nav_msgs::msg::Path>(
    "path", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  nav_msgs::msg::Path bad;
  bad.header.stamp = node_->now();
  bad.header.frame_id = "map";  // != default world_frame "world"
  geometry_msgs::msg::PoseStamped pose;
  pose.header = bad.header;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 1.5;
  bad.poses.push_back(pose);

  path_pub->publish(bad);
  rclcpp::spin_some(node_);
  // Allow subscription callback
  for (int i = 0; i < 20; ++i) {
    rclcpp::spin_some(node_);
    if (node_->pathRejectCount() > 0u) {
      break;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_EQ(node_->pathRejectCount(), 1u);
  EXPECT_TRUE(node_->hasActivePath());
}

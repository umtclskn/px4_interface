#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "px4_interface/path_hold_policy.hpp"

namespace
{

using px4_interface::ActivePathState;
using px4_interface::FollowDecision;
using px4_interface::FollowDecisionInput;
using px4_interface::FollowDecisionKind;
using px4_interface::HoldLatch;
using px4_interface::PathUpdateKind;
using px4_interface::Vec3;
using px4_interface::applyPathUpdate;
using px4_interface::decideFollowOrHold;

constexpr double kEps = 1e-9;

FollowDecisionInput baseFollowInput()
{
  FollowDecisionInput in;
  in.now_s = 10.0;
  in.path.has_path = true;
  in.path.received_s = 9.5;
  in.path_timeout_s = 3.0;
  in.pose_valid = true;
  in.pose_position = Vec3{1.0, 2.0, -1.5};
  in.tracker_setpoint_valid = true;
  in.tracker_setpoint = Vec3{1.5, 2.0, -1.5};
  return in;
}

}  // namespace

TEST(PathHoldPolicyTest, AcceptedPathFollows)
{
  HoldLatch latch;
  const FollowDecision d = decideFollowOrHold(baseFollowInput(), latch);
  EXPECT_EQ(d.kind, FollowDecisionKind::kFollow);
  EXPECT_NEAR(d.setpoint.x, 1.5, kEps);
  EXPECT_FALSE(latch.latched);
}

TEST(PathHoldPolicyTest, EmptyPathHolds)
{
  ActivePathState state;
  applyPathUpdate(state, PathUpdateKind::kAccepted, 1.0);
  applyPathUpdate(state, PathUpdateKind::kEmpty, 2.0);
  EXPECT_FALSE(state.has_path);

  FollowDecisionInput in = baseFollowInput();
  in.path = state;
  in.now_s = 2.0;
  HoldLatch latch;
  const FollowDecision d = decideFollowOrHold(in, latch);
  EXPECT_EQ(d.kind, FollowDecisionKind::kHold);
  EXPECT_TRUE(latch.latched);
  EXPECT_NEAR(d.setpoint.x, in.pose_position.x, kEps);
}

TEST(PathHoldPolicyTest, StalePathHolds)
{
  FollowDecisionInput in = baseFollowInput();
  in.path.received_s = 0.0;
  in.path_timeout_s = 3.0;
  HoldLatch latch;

  in.now_s = 3.0 + 1e-6;
  EXPECT_EQ(
    decideFollowOrHold(in, latch).kind, FollowDecisionKind::kHold);

  latch = HoldLatch{};
  in.now_s = 3.0 - 1e-6;
  EXPECT_EQ(
    decideFollowOrHold(in, latch).kind, FollowDecisionKind::kFollow);
}

TEST(PathHoldPolicyTest, RejectedPathKeepsPrevious)
{
  ActivePathState state;
  applyPathUpdate(state, PathUpdateKind::kAccepted, 5.0);
  applyPathUpdate(state, PathUpdateKind::kRejected, 6.0);
  EXPECT_TRUE(state.has_path);
  EXPECT_NEAR(state.received_s, 5.0, kEps);
  EXPECT_EQ(state.reject_count, 1u);
}

TEST(PathHoldPolicyTest, RejectedPathStillAges)
{
  ActivePathState state;
  applyPathUpdate(state, PathUpdateKind::kAccepted, 1.0);
  applyPathUpdate(state, PathUpdateKind::kRejected, 1.5);

  FollowDecisionInput in = baseFollowInput();
  in.path = state;
  in.path_timeout_s = 2.0;
  in.now_s = 3.0 + 1e-6;
  HoldLatch latch;
  EXPECT_EQ(
    decideFollowOrHold(in, latch).kind, FollowDecisionKind::kHold);
  EXPECT_EQ(in.path.reject_count, 1u);
}

TEST(PathHoldPolicyTest, HoldLatchesOnEntry)
{
  FollowDecisionInput in = baseFollowInput();
  in.path.has_path = false;
  HoldLatch latch;
  const FollowDecision d = decideFollowOrHold(in, latch);
  EXPECT_EQ(d.kind, FollowDecisionKind::kHold);
  EXPECT_TRUE(latch.latched);
  EXPECT_NEAR(latch.position.x, 1.0, kEps);
  EXPECT_NEAR(d.setpoint.x, 1.0, kEps);
}

TEST(PathHoldPolicyTest, HoldLatchNotUpdated)
{
  FollowDecisionInput in = baseFollowInput();
  in.path.has_path = false;
  HoldLatch latch;
  ASSERT_EQ(decideFollowOrHold(in, latch).kind, FollowDecisionKind::kHold);

  in.pose_position = Vec3{9.0, 9.0, -9.0};
  const FollowDecision d = decideFollowOrHold(in, latch);
  EXPECT_EQ(d.kind, FollowDecisionKind::kHold);
  EXPECT_NEAR(d.setpoint.x, 1.0, kEps);
  EXPECT_NEAR(latch.position.x, 1.0, kEps);
}

TEST(PathHoldPolicyTest, FollowClearsLatch)
{
  FollowDecisionInput in = baseFollowInput();
  in.path.has_path = false;
  HoldLatch latch;
  ASSERT_EQ(decideFollowOrHold(in, latch).kind, FollowDecisionKind::kHold);
  ASSERT_TRUE(latch.latched);

  in = baseFollowInput();
  ASSERT_EQ(decideFollowOrHold(in, latch).kind, FollowDecisionKind::kFollow);
  EXPECT_FALSE(latch.latched);

  in.path.has_path = false;
  in.pose_position = Vec3{4.0, 5.0, -1.0};
  const FollowDecision d = decideFollowOrHold(in, latch);
  EXPECT_EQ(d.kind, FollowDecisionKind::kHold);
  EXPECT_NEAR(d.setpoint.x, 4.0, kEps);
}

TEST(PathHoldPolicyTest, NonFiniteSetpointHolds)
{
  FollowDecisionInput in = baseFollowInput();
  in.tracker_setpoint = Vec3{
    std::numeric_limits<double>::quiet_NaN(), 0.0, -1.0};
  HoldLatch latch;
  const FollowDecision d = decideFollowOrHold(in, latch);
  EXPECT_EQ(d.kind, FollowDecisionKind::kHold);
  EXPECT_TRUE(std::isfinite(d.setpoint.x));
  EXPECT_TRUE(std::isfinite(d.setpoint.y));
  EXPECT_TRUE(std::isfinite(d.setpoint.z));
}

TEST(PathHoldPolicyTest, TimeBackwardsHolds)
{
  FollowDecisionInput in = baseFollowInput();
  in.now_s = 1.0;
  in.path.received_s = 5.0;
  HoldLatch latch;
  EXPECT_EQ(
    decideFollowOrHold(in, latch).kind, FollowDecisionKind::kHold);
}

TEST(PathHoldPolicyTest, NoPoseNoLatchNoSetpoint)
{
  FollowDecisionInput in = baseFollowInput();
  in.path.has_path = false;
  in.pose_valid = false;
  HoldLatch latch;
  const FollowDecision d = decideFollowOrHold(in, latch);
  EXPECT_EQ(d.kind, FollowDecisionKind::kNoSetpoint);
  EXPECT_FALSE(latch.latched);
}

TEST(PathHoldPolicyTest, InvalidPoseNeverLatchesOrigin)
{
  FollowDecisionInput in = baseFollowInput();
  in.path.has_path = false;
  in.pose_valid = false;
  in.pose_position = Vec3{0.0, 0.0, 0.0};
  HoldLatch latch;
  const FollowDecision d = decideFollowOrHold(in, latch);
  EXPECT_EQ(d.kind, FollowDecisionKind::kNoSetpoint);
  EXPECT_FALSE(latch.latched);
}

TEST(PathHoldPolicyTest, HoldKeepsLatchWhenPoseBecomesInvalid)
{
  FollowDecisionInput in = baseFollowInput();
  in.path.has_path = false;
  HoldLatch latch;
  ASSERT_EQ(decideFollowOrHold(in, latch).kind, FollowDecisionKind::kHold);
  ASSERT_TRUE(latch.latched);

  in.pose_valid = false;
  in.pose_position = Vec3{0.0, 0.0, 0.0};
  const FollowDecision d = decideFollowOrHold(in, latch);
  EXPECT_EQ(d.kind, FollowDecisionKind::kHold);
  EXPECT_NEAR(d.setpoint.x, 1.0, kEps);
  EXPECT_NEAR(latch.position.x, 1.0, kEps);
}

#pragma once

#include <cstddef>
#include <string>

#include "px4_interface/frame_conversions.hpp"

namespace px4_interface
{

enum class PathUpdateKind
{
  kAccepted,
  kEmpty,
  kRejected,
};

struct ActivePathState
{
  bool has_path{false};
  double received_s{0.0};
  std::size_t reject_count{0};
};

/// Applies one incoming path message outcome (decided by the node:
/// frame_id check, worldToNed, validatePathNed).
void applyPathUpdate(
  ActivePathState & state, PathUpdateKind kind, double now_s);

enum class FollowDecisionKind
{
  kFollow,
  kHold,
  kNoSetpoint,
};

struct FollowDecisionInput
{
  double now_s{0.0};
  ActivePathState path;
  double path_timeout_s{0.0};
  bool pose_valid{false};
  Vec3 pose_position;
  bool tracker_setpoint_valid{false};
  Vec3 tracker_setpoint;
};

struct HoldLatch
{
  bool latched{false};
  Vec3 position;
};

struct FollowDecision
{
  FollowDecisionKind kind{FollowDecisionKind::kNoSetpoint};
  Vec3 setpoint;
  std::string detail;
};

FollowDecision decideFollowOrHold(
  const FollowDecisionInput & input, HoldLatch & latch);

}  // namespace px4_interface

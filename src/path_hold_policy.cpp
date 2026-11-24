#include "px4_interface/path_hold_policy.hpp"

#include <cmath>

namespace px4_interface
{
namespace
{

FollowDecision holdOrNoSetpoint(
  const FollowDecisionInput & input,
  HoldLatch & latch,
  const std::string & detail)
{
  FollowDecision out;
  out.detail = detail;

  if (latch.latched && isFinite(latch.position)) {
    out.kind = FollowDecisionKind::kHold;
    out.setpoint = latch.position;
    return out;
  }

  if (input.pose_valid && isFinite(input.pose_position)) {
    latch.latched = true;
    latch.position = input.pose_position;
    out.kind = FollowDecisionKind::kHold;
    out.setpoint = latch.position;
    return out;
  }

  out.kind = FollowDecisionKind::kNoSetpoint;
  out.setpoint = Vec3{};
  return out;
}

}  // namespace

void applyPathUpdate(
  ActivePathState & state, const PathUpdateKind kind, const double now_s)
{
  switch (kind) {
    case PathUpdateKind::kAccepted:
      state.has_path = true;
      state.received_s = now_s;
      break;
    case PathUpdateKind::kEmpty:
      state.has_path = false;
      break;
    case PathUpdateKind::kRejected:
      ++state.reject_count;
      break;
  }
}

FollowDecision decideFollowOrHold(
  const FollowDecisionInput & input, HoldLatch & latch)
{
  if (!std::isfinite(input.now_s)) {
    return holdOrNoSetpoint(input, latch, "non_finite_now");
  }
  if (!(input.path_timeout_s > 0.0) || !std::isfinite(input.path_timeout_s)) {
    return holdOrNoSetpoint(input, latch, "invalid_path_timeout");
  }

  const bool age_ok =
    input.path.has_path &&
    std::isfinite(input.path.received_s) &&
    (input.now_s >= input.path.received_s) &&
    ((input.now_s - input.path.received_s) <= input.path_timeout_s);

  const bool can_follow =
    age_ok &&
    input.pose_valid &&
    input.tracker_setpoint_valid &&
    isFinite(input.tracker_setpoint);

  if (can_follow) {
    latch.latched = false;
    FollowDecision out;
    out.kind = FollowDecisionKind::kFollow;
    out.setpoint = input.tracker_setpoint;
    out.detail = "follow";
    return out;
  }

  std::string detail = "hold";
  if (!input.path.has_path) {
    detail = "no_path";
  } else if (!std::isfinite(input.path.received_s)) {
    detail = "non_finite_received_s";
  } else if (input.now_s < input.path.received_s) {
    detail = "time_backwards";
  } else if ((input.now_s - input.path.received_s) > input.path_timeout_s) {
    detail = "path_stale";
  } else if (!input.pose_valid) {
    detail = "pose_invalid";
  } else if (!input.tracker_setpoint_valid ||
    !isFinite(input.tracker_setpoint))
  {
    detail = "tracker_setpoint_invalid";
  }

  return holdOrNoSetpoint(input, latch, detail);
}

}  // namespace px4_interface

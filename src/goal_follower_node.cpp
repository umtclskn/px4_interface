#include "px4_interface/goal_follower_node.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

using namespace std::chrono_literals;

namespace px4_interface
{
namespace
{

rclcpp::QoS px4SensorQos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

rclcpp::QoS px4CommandQos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

rclcpp::QoS pathQos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

double elapsedSeconds(const rclcpp::Time & start, const rclcpp::Time & now)
{
  return (now - start).seconds();
}

float yawFromPx4Quaternion(const std::array<float, 4> & q)
{
  // PX4: q = (w, x, y, z). Yaw about NED down axis.
  const float w = q[0];
  const float x = q[1];
  const float y = q[2];
  const float z = q[3];
  return std::atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
}

bool requirePositive(const double value, const char * /*name*/)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

GoalFollowerNode::GoalFollowerNode(const rclcpp::NodeOptions & options)
: Node("goal_follower_node", options),
  state_entered_(now()),
  last_command_sent_(now()),
  landing_started_(now()),
  last_path_stamp_(now())
{
  world_frame_ = declare_parameter("world_frame", "world");
  body_frame_ = declare_parameter("body_frame", "body");
  world_yaw_offset_rad_ = declare_parameter("world_yaw_offset_rad", 0.0);
  takeoff_height_m_ = declare_parameter("takeoff_height_m", 1.5);
  lookahead_m_ = declare_parameter("lookahead_m", 1.0);
  position_tolerance_m_ = declare_parameter("position_tolerance_m", 0.2);
  path_timeout_s_ = declare_parameter("path_timeout_s", 3.0);
  path_limits_.max_start_distance_m =
    declare_parameter("max_start_distance_m", 3.0);
  path_limits_.max_horizontal_distance_m =
    declare_parameter("max_horizontal_distance_m", 50.0);
  path_limits_.min_altitude_m = declare_parameter("min_altitude_m", 0.5);
  path_limits_.max_altitude_m = declare_parameter("max_altitude_m", 5.0);
  yaw_mode_ = declare_parameter("yaw_mode", "hold");
  px4_ready_timeout_s_ = declare_parameter("px4_ready_timeout_s", 30.0);
  prestream_duration_s_ = declare_parameter("prestream_duration_s", 1.0);
  prestream_timeout_s_ = declare_parameter("prestream_timeout_s", 10.0);
  offboard_timeout_s_ = declare_parameter("offboard_timeout_s", 10.0);
  arm_timeout_s_ = declare_parameter("arm_timeout_s", 10.0);
  landing_timeout_s_ = declare_parameter("landing_timeout_s", 30.0);

  if (world_frame_.empty() || body_frame_.empty()) {
    throw std::invalid_argument("world_frame and body_frame must be non-empty");
  }
  if (!std::isfinite(world_yaw_offset_rad_)) {
    throw std::invalid_argument("world_yaw_offset_rad must be finite");
  }
  if (!requirePositive(takeoff_height_m_, "takeoff_height_m") ||
    !requirePositive(lookahead_m_, "lookahead_m") ||
    !requirePositive(position_tolerance_m_, "position_tolerance_m") ||
    !requirePositive(path_timeout_s_, "path_timeout_s") ||
    !requirePositive(path_limits_.max_start_distance_m, "max_start_distance_m") ||
    !requirePositive(
      path_limits_.max_horizontal_distance_m, "max_horizontal_distance_m") ||
    !requirePositive(path_limits_.min_altitude_m, "min_altitude_m") ||
    !requirePositive(path_limits_.max_altitude_m, "max_altitude_m") ||
    !(path_limits_.max_altitude_m > path_limits_.min_altitude_m) ||
    !requirePositive(px4_ready_timeout_s_, "px4_ready_timeout_s") ||
    !requirePositive(prestream_duration_s_, "prestream_duration_s") ||
    !requirePositive(prestream_timeout_s_, "prestream_timeout_s") ||
    !requirePositive(offboard_timeout_s_, "offboard_timeout_s") ||
    !requirePositive(arm_timeout_s_, "arm_timeout_s") ||
    !requirePositive(landing_timeout_s_, "landing_timeout_s"))
  {
    throw std::invalid_argument("goal_follower_node parameter validation failed");
  }
  if (yaw_mode_ != "hold") {
    throw std::invalid_argument("yaw_mode must be 'hold' in this task");
  }
  if (prestream_duration_s_ < 1.0) {
    RCLCPP_WARN(
      get_logger(),
      "prestream_duration_s=%.3f is below 1.0 s; clamping to 1.0 s",
      prestream_duration_s_);
    prestream_duration_s_ = 1.0;
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  offboard_control_mode_pub_ =
    create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", px4CommandQos());
  trajectory_setpoint_pub_ =
    create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", px4CommandQos());
  vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", px4CommandQos());

  vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status_v4", px4SensorQos(),
    std::bind(&GoalFollowerNode::onVehicleStatus, this, std::placeholders::_1));
  vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry", px4SensorQos(),
    std::bind(
      &GoalFollowerNode::onVehicleOdometry, this, std::placeholders::_1));
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "path", pathQos(),
    std::bind(&GoalFollowerNode::onPath, this, std::placeholders::_1));
  land_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/land",
    std::bind(
      &GoalFollowerNode::onLand, this, std::placeholders::_1,
      std::placeholders::_2));

  timer_ = create_wall_timer(
    50ms, std::bind(&GoalFollowerNode::timerCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "goal_follower_node started (world=%s body=%s takeoff=%.2f m lookahead=%.2f m)",
    world_frame_.c_str(), body_frame_.c_str(), takeoff_height_m_, lookahead_m_);
}

void GoalFollowerNode::onVehicleStatus(
  const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  last_status_ = *msg;
  if (msg->system_id != 0) {
    target_system_ = msg->system_id;
  }
  if (msg->component_id != 0) {
    target_component_ = msg->component_id;
  }
}

void GoalFollowerNode::onVehicleOdometry(
  const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  last_odometry_ = *msg;
}

void GoalFollowerNode::onLand(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  land_requested_ = true;
  response->success = true;
  response->message = "landing requested";
  RCLCPP_INFO(get_logger(), "Land service requested");
}

void GoalFollowerNode::onPath(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->header.frame_id != world_frame_) {
    ++path_reject_count_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "path rejected: frame_id='%s' != world_frame='%s' (rejects=%zu)",
      msg->header.frame_id.c_str(), world_frame_.c_str(), path_reject_count_);
    return;
  }
  if (msg->poses.size() > path_limits_.max_points) {
    ++path_reject_count_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "path rejected: too many points (%zu > %zu)",
      msg->poses.size(), path_limits_.max_points);
    return;
  }

  if (msg->poses.empty()) {
    path_tracker_.clear();
    last_path_stamp_ = now();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "empty path accepted -> hold");
    return;
  }

  if (!alignment_locked_ || !odometryNedValid()) {
    ++path_reject_count_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "path rejected: alignment/odometry not ready");
    return;
  }

  std::vector<Vec3> path_ned;
  path_ned.reserve(msg->poses.size());
  for (const auto & pose : msg->poses) {
    Vec3 world;
    world.x = pose.pose.position.x;
    world.y = pose.pose.position.y;
    world.z = pose.pose.position.z;
    const auto ned = worldToNed(world, alignment_);
    if (!ned.has_value()) {
      ++path_reject_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "path rejected: worldToNed failed (non-finite)");
      return;
    }
    path_ned.push_back(*ned);
  }

  const PathRejection rejection = validatePathNed(
    path_ned, vehicleNed(), alignment_.ned_home, path_limits_);
  if (rejection != PathRejection::kNone) {
    ++path_reject_count_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "path rejected: %s (rejects=%zu)",
      pathRejectionName(rejection), path_reject_count_);
    return;
  }

  path_tracker_.setPath(path_ned);
  last_path_stamp_ = now();
}

bool GoalFollowerNode::odometryNedValid() const
{
  if (!last_odometry_.has_value()) {
    return false;
  }
  if (last_odometry_->pose_frame !=
    px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED)
  {
    return false;
  }
  const auto & p = last_odometry_->position;
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

Vec3 GoalFollowerNode::vehicleNed() const
{
  Vec3 v;
  if (!odometryNedValid()) {
    return v;
  }
  v.x = static_cast<double>(last_odometry_->position[0]);
  v.y = static_cast<double>(last_odometry_->position[1]);
  v.z = static_cast<double>(last_odometry_->position[2]);
  return v;
}

Vec3 GoalFollowerNode::takeoffSetpointNed() const
{
  Vec3 target = alignment_.ned_home;
  target.z = alignment_.ned_home.z - takeoff_height_m_;
  return target;
}

bool GoalFollowerNode::withinTolerance(const Vec3 & target_ned) const
{
  if (!odometryNedValid()) {
    return false;
  }
  return distance(vehicleNed(), target_ned) <= position_tolerance_m_;
}

bool GoalFollowerNode::pathUsable(const rclcpp::Time & t) const
{
  if (!path_tracker_.hasPath()) {
    return false;
  }
  return elapsedSeconds(last_path_stamp_, t) <= path_timeout_s_;
}

bool GoalFollowerNode::tryCaptureAlignment()
{
  if (alignment_locked_) {
    return true;
  }
  if (!isDisarmed() || !odometryNedValid()) {
    return false;
  }
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      world_frame_, body_frame_, tf2::TimePointZero);
  } catch (const std::exception & ex) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TF %s->%s unavailable: %s", world_frame_.c_str(), body_frame_.c_str(),
      ex.what());
    return false;
  }
  if (!std::isfinite(tf.transform.translation.x) ||
    !std::isfinite(tf.transform.translation.y) ||
    !std::isfinite(tf.transform.translation.z))
  {
    return false;
  }

  alignment_.world_home.x = tf.transform.translation.x;
  alignment_.world_home.y = tf.transform.translation.y;
  alignment_.world_home.z = tf.transform.translation.z;
  alignment_.ned_home = vehicleNed();
  alignment_.world_yaw_offset_rad = world_yaw_offset_rad_;
  alignment_.valid = true;
  hold_yaw_ned_ = yawFromPx4Quaternion(last_odometry_->q);
  if (!std::isfinite(hold_yaw_ned_)) {
    hold_yaw_ned_ = 0.0f;
  }
  alignment_locked_ = true;
  setpoint_ned_ = takeoffSetpointNed();
  RCLCPP_INFO(
    get_logger(),
    "Alignment locked: world[%.3f, %.3f, %.3f] ned[%.3f, %.3f, %.3f] yaw_ned=%.3f",
    alignment_.world_home.x, alignment_.world_home.y, alignment_.world_home.z,
    alignment_.ned_home.x, alignment_.ned_home.y, alignment_.ned_home.z,
    static_cast<double>(hold_yaw_ned_));
  return true;
}

void GoalFollowerNode::enterHold(const std::string & reason)
{
  if (state_ != GoalFollowerState::kHold) {
    hold_ned_ = vehicleNed();
    hold_latched_ = true;
    setpoint_ned_ = hold_ned_;
    transitionTo(GoalFollowerState::kHold, reason);
  }
}

void GoalFollowerNode::enterFollow(const std::string & reason)
{
  hold_latched_ = false;
  if (state_ != GoalFollowerState::kFollow) {
    transitionTo(GoalFollowerState::kFollow, reason);
  }
}

void GoalFollowerNode::updateFollowHold(const rclcpp::Time & t)
{
  if (pathUsable(t)) {
    if (state_ == GoalFollowerState::kHold) {
      enterFollow("path available");
    }
  } else if (state_ == GoalFollowerState::kFollow) {
    enterHold("path missing/timeout/empty");
  } else if (state_ == GoalFollowerState::kTakeoff &&
    withinTolerance(takeoffSetpointNed()))
  {
    if (pathUsable(t)) {
      enterFollow("takeoff complete with path");
    } else {
      enterHold("takeoff complete, no path");
    }
  }
}

void GoalFollowerNode::timerCallback()
{
  const rclcpp::Time t = now();
  const double in_state_s = elapsedSeconds(state_entered_, t);

  if (land_requested_ &&
    (state_ == GoalFollowerState::kTakeoff ||
    state_ == GoalFollowerState::kFollow ||
    state_ == GoalFollowerState::kHold ||
    state_ == GoalFollowerState::kRequestArm ||
    state_ == GoalFollowerState::kRequestOffboard ||
    state_ == GoalFollowerState::kPrestreamSetpoints))
  {
    if (isArmed()) {
      enterRequestLand("land service");
    } else {
      finishAbortedDisarmed("land service while disarmed");
    }
    land_requested_ = false;
  }

  switch (state_) {
    case GoalFollowerState::kWaitForPx4:
      // No setpoints before alignment (spec).
      if (hasFreshStatus()) {
        transitionTo(GoalFollowerState::kWaitForAlignment, "vehicle_status received");
      } else if (in_state_s >= px4_ready_timeout_s_) {
        finishAbortedDisarmed("px4_ready_timeout_s expired with no VehicleStatus");
      }
      break;

    case GoalFollowerState::kWaitForAlignment:
      if (isArmed()) {
        enterRequestLand("armed during alignment wait");
        break;
      }
      if (tryCaptureAlignment()) {
        transitionTo(
          GoalFollowerState::kPrestreamSetpoints, "world/NED alignment locked");
      } else if (in_state_s >= px4_ready_timeout_s_) {
        finishAbortedDisarmed("alignment timeout (TF or NED odometry)");
      }
      break;

    case GoalFollowerState::kPrestreamSetpoints:
      if (isArmed()) {
        enterRequestLand("armed during prestream");
        break;
      }
      if (!publishStreamingSetpoints(takeoffSetpointNed())) {
        finishAbortedDisarmed("non-finite takeoff setpoint in prestream");
        break;
      }
      if (in_state_s >= prestream_duration_s_) {
        transitionTo(GoalFollowerState::kRequestOffboard, "prestream complete");
      } else if (in_state_s >= prestream_timeout_s_) {
        finishAbortedDisarmed("prestream_timeout_s expired");
      }
      break;

    case GoalFollowerState::kRequestOffboard:
      publishStreamingSetpoints(takeoffSetpointNed());
      if (isOffboard()) {
        transitionTo(GoalFollowerState::kRequestArm, "nav_state OFFBOARD");
      } else if (in_state_s >= offboard_timeout_s_) {
        if (isArmed()) {
          enterRequestLand("offboard_timeout_s expired while armed");
        } else {
          finishAbortedDisarmed("offboard_timeout_s expired while disarmed");
        }
      } else if (maybeRetryCommand()) {
        requestOffboardMode();
      }
      break;

    case GoalFollowerState::kRequestArm:
      publishStreamingSetpoints(takeoffSetpointNed());
      if (!isOffboard()) {
        if (isArmed()) {
          enterRequestLand("offboard lost while requesting arm");
        } else {
          transitionTo(
            GoalFollowerState::kRequestOffboard, "offboard lost before arm");
        }
      } else if (isArmed()) {
        transitionTo(GoalFollowerState::kTakeoff, "arming_state ARMED");
      } else if (in_state_s >= arm_timeout_s_) {
        if (isArmed()) {
          enterRequestLand("arm_timeout_s expired while armed");
        } else {
          finishAbortedDisarmed("arm_timeout_s expired while disarmed");
        }
      } else if (maybeRetryCommand()) {
        requestArm();
      }
      break;

    case GoalFollowerState::kTakeoff:
    case GoalFollowerState::kFollow:
    case GoalFollowerState::kHold:
      {
        if (last_status_.has_value() && last_status_->failsafe) {
          if (isArmed()) {
            enterRequestLand("PX4 failsafe reported");
          } else {
            finishAbortedDisarmed("PX4 failsafe while disarmed");
          }
          break;
        }
        if (!isOffboard() || !isArmed()) {
          if (isArmed()) {
            enterRequestLand("offboard or arming feedback lost");
          } else {
            finishAbortedDisarmed("disarmed during flight");
          }
          break;
        }

        if (state_ == GoalFollowerState::kTakeoff) {
          setpoint_ned_ = takeoffSetpointNed();
          if (withinTolerance(setpoint_ned_)) {
            updateFollowHold(t);
          }
        } else {
          updateFollowHold(t);
          if (state_ == GoalFollowerState::kFollow) {
            const LookaheadResult look =
              path_tracker_.update(vehicleNed(), lookahead_m_);
            if (!look.valid) {
              enterHold("path tracker invalid");
            } else {
              setpoint_ned_ = look.target;
            }
          } else if (state_ == GoalFollowerState::kHold) {
            if (!hold_latched_) {
              hold_ned_ = vehicleNed();
              hold_latched_ = true;
            }
            setpoint_ned_ = hold_ned_;
          }
        }

        if (!publishStreamingSetpoints(setpoint_ned_)) {
          enterHold("non-finite setpoint");
        }
      }
      break;

    case GoalFollowerState::kRequestLand:
      if (isDisarmed()) {
        transitionTo(GoalFollowerState::kWaitDisarm, "already DISARMED");
      } else if (isAutoLand()) {
        transitionTo(GoalFollowerState::kWaitDisarm, "nav_state AUTO_LAND");
      } else if (landingTimedOut(t) && isArmed()) {
        transitionTo(
          GoalFollowerState::kFailedLanding,
          "landing_timeout_s expired while armed");
      } else if (maybeRetryCommand()) {
        requestLand();
      }
      break;

    case GoalFollowerState::kWaitDisarm:
      if (isDisarmed()) {
        transitionTo(GoalFollowerState::kDone, "arming_state DISARMED");
      } else if (landingTimedOut(t) && isArmed()) {
        transitionTo(
          GoalFollowerState::kFailedLanding,
          "landing_timeout_s expired while armed");
      } else if (!isAutoLand() && !isDisarmed() && maybeRetryCommand()) {
        requestLand();
      }
      break;

    case GoalFollowerState::kDone:
    case GoalFollowerState::kAbortedDisarmed:
    case GoalFollowerState::kFailedLanding:
      break;
  }
}

void GoalFollowerNode::publishOffboardControlMode()
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = timestampUs();
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.thrust_and_torque = false;
  msg.direct_actuator = false;
  offboard_control_mode_pub_->publish(msg);
}

bool GoalFollowerNode::publishTrajectorySetpoint(const Vec3 & target_ned)
{
  if (!isFinite(target_ned) || !std::isfinite(hold_yaw_ned_)) {
    return false;
  }
  const float fx = static_cast<float>(target_ned.x);
  const float fy = static_cast<float>(target_ned.y);
  const float fz = static_cast<float>(target_ned.z);
  if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(fz)) {
    return false;
  }
  const float nan = std::numeric_limits<float>::quiet_NaN();
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.timestamp = timestampUs();
  msg.position = {fx, fy, fz};
  msg.velocity = {nan, nan, nan};
  msg.acceleration = {nan, nan, nan};
  msg.jerk = {nan, nan, nan};
  msg.yaw = hold_yaw_ned_;
  msg.yawspeed = nan;
  trajectory_setpoint_pub_->publish(msg);
  return true;
}

bool GoalFollowerNode::publishStreamingSetpoints(const Vec3 & target_ned)
{
  if (!alignment_locked_) {
    return false;
  }
  publishOffboardControlMode();
  return publishTrajectorySetpoint(target_ned);
}

void GoalFollowerNode::publishVehicleCommand(
  const uint32_t command, const float param1, const float param2)
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = timestampUs();
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = target_system_;
  msg.target_component = target_component_;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.confirmation = 0;
  msg.from_external = true;
  vehicle_command_pub_->publish(msg);
  last_command_sent_ = now();
  command_sent_in_state_ = true;
}

void GoalFollowerNode::requestOffboardMode()
{
  publishVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
  RCLCPP_INFO(get_logger(), "Sent VEHICLE_CMD_DO_SET_MODE (custom=1, offboard=6)");
}

void GoalFollowerNode::requestArm()
{
  publishVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    static_cast<float>(px4_msgs::msg::VehicleCommand::ARMING_ACTION_ARM),
    0.0f);
  RCLCPP_INFO(get_logger(), "Sent VEHICLE_CMD_COMPONENT_ARM_DISARM (arm)");
}

void GoalFollowerNode::requestLand()
{
  publishVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0f, 0.0f);
  RCLCPP_INFO(get_logger(), "Sent VEHICLE_CMD_NAV_LAND");
}

void GoalFollowerNode::transitionTo(
  const GoalFollowerState next, const std::string & reason)
{
  if (next == GoalFollowerState::kFailedLanding ||
    next == GoalFollowerState::kAbortedDisarmed)
  {
    RCLCPP_ERROR(
      get_logger(), "STATE %s -> %s (%s)",
      stateName(state_), stateName(next), reason.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(), "STATE %s -> %s (%s)",
      stateName(state_), stateName(next), reason.c_str());
  }
  state_ = next;
  state_entered_ = now();
  command_sent_in_state_ = false;
}

void GoalFollowerNode::finishAbortedDisarmed(const std::string & reason)
{
  transitionTo(GoalFollowerState::kAbortedDisarmed, reason);
}

void GoalFollowerNode::enterRequestLand(const std::string & reason)
{
  landing_started_ = now();
  transitionTo(GoalFollowerState::kRequestLand, reason);
}

bool GoalFollowerNode::hasFreshStatus() const
{
  return last_status_.has_value();
}

bool GoalFollowerNode::isArmed() const
{
  return last_status_.has_value() &&
    last_status_->arming_state ==
    px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
}

bool GoalFollowerNode::isDisarmed() const
{
  return last_status_.has_value() &&
    last_status_->arming_state ==
    px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
}

bool GoalFollowerNode::isOffboard() const
{
  return last_status_.has_value() &&
    last_status_->nav_state ==
    px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

bool GoalFollowerNode::isAutoLand() const
{
  return last_status_.has_value() &&
    last_status_->nav_state ==
    px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
}

bool GoalFollowerNode::landingTimedOut(const rclcpp::Time & t) const
{
  return elapsedSeconds(landing_started_, t) >= landing_timeout_s_;
}

uint64_t GoalFollowerNode::timestampUs() const
{
  return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
}

bool GoalFollowerNode::maybeRetryCommand()
{
  if (!command_sent_in_state_) {
    return true;
  }
  return elapsedSeconds(last_command_sent_, now()) >= 1.0;
}

const char * GoalFollowerNode::stateName(const GoalFollowerState state)
{
  switch (state) {
    case GoalFollowerState::kWaitForPx4:
      return "WAIT_FOR_PX4";
    case GoalFollowerState::kWaitForAlignment:
      return "WAIT_FOR_ALIGNMENT";
    case GoalFollowerState::kPrestreamSetpoints:
      return "PRESTREAM_SETPOINTS";
    case GoalFollowerState::kRequestOffboard:
      return "REQUEST_OFFBOARD";
    case GoalFollowerState::kRequestArm:
      return "REQUEST_ARM";
    case GoalFollowerState::kTakeoff:
      return "TAKEOFF";
    case GoalFollowerState::kFollow:
      return "FOLLOW";
    case GoalFollowerState::kHold:
      return "HOLD";
    case GoalFollowerState::kRequestLand:
      return "REQUEST_LAND";
    case GoalFollowerState::kWaitDisarm:
      return "WAIT_DISARM";
    case GoalFollowerState::kDone:
      return "DONE";
    case GoalFollowerState::kAbortedDisarmed:
      return "ABORTED_DISARMED";
    case GoalFollowerState::kFailedLanding:
      return "FAILED_LANDING";
  }
  return "UNKNOWN";
}

const char * GoalFollowerNode::pathRejectionName(const PathRejection rejection)
{
  switch (rejection) {
    case PathRejection::kNone:
      return "none";
    case PathRejection::kEmpty:
      return "empty";
    case PathRejection::kTooManyPoints:
      return "too_many_points";
    case PathRejection::kNonFinite:
      return "non_finite";
    case PathRejection::kStartTooFar:
      return "start_too_far";
    case PathRejection::kOutsideGeofence:
      return "outside_geofence";
  }
  return "unknown";
}

}  // namespace px4_interface

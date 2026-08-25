#include "px4_interface/offboard_hover_node.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

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

double elapsedSeconds(const rclcpp::Time & start, const rclcpp::Time & now)
{
  return (now - start).seconds();
}

}  // namespace

OffboardHoverNode::OffboardHoverNode()
: Node("offboard_hover_node"),
  state_entered_(now()),
  last_command_sent_(now()),
  landing_started_(now()),
  home_locked_at_(now()),
  target_reached_at_(now())
{
  hover_duration_s_ = declare_parameter("hover_duration_s", 10.0);
  px4_ready_timeout_s_ = declare_parameter("px4_ready_timeout_s", 30.0);
  prestream_duration_s_ = declare_parameter("prestream_duration_s", 1.0);
  prestream_timeout_s_ = declare_parameter("prestream_timeout_s", 10.0);
  offboard_timeout_s_ = declare_parameter("offboard_timeout_s", 10.0);
  arm_timeout_s_ = declare_parameter("arm_timeout_s", 10.0);
  landing_timeout_s_ = declare_parameter("landing_timeout_s", 30.0);
  target_x_ = static_cast<float>(declare_parameter("target_x", 0.0));
  target_y_ = static_cast<float>(declare_parameter("target_y", 0.0));
  target_z_ = static_cast<float>(declare_parameter("target_z", -2.0));
  target_yaw_ = static_cast<float>(declare_parameter("target_yaw", 0.0));
  use_relative_target_ = declare_parameter("use_relative_target", true);
  takeoff_height_m_ = declare_parameter("takeoff_height_m", 1.0);
  relative_offset_x_m_ = declare_parameter("relative_offset_x_m", 0.0);
  relative_offset_y_m_ = declare_parameter("relative_offset_y_m", 0.0);
  position_tolerance_m_ = declare_parameter("position_tolerance_m", 0.15);
  target_reach_timeout_s_ = declare_parameter("target_reach_timeout_s", 45.0);

  if (prestream_duration_s_ < 1.0) {
    RCLCPP_WARN(
      get_logger(),
      "prestream_duration_s=%.3f is below 1.0 s; clamping to 1.0 s",
      prestream_duration_s_);
    prestream_duration_s_ = 1.0;
  }

  if (position_tolerance_m_ <= 0.0) {
    RCLCPP_FATAL(
      get_logger(),
      "position_tolerance_m must be > 0 (got %.3f); refusing to start",
      position_tolerance_m_);
    throw std::invalid_argument("position_tolerance_m must be > 0");
  }
  if (target_reach_timeout_s_ <= 0.0) {
    RCLCPP_FATAL(
      get_logger(),
      "target_reach_timeout_s must be > 0 (got %.3f); refusing to start",
      target_reach_timeout_s_);
    throw std::invalid_argument("target_reach_timeout_s must be > 0");
  }
  if (use_relative_target_ && takeoff_height_m_ <= 0.0) {
    RCLCPP_FATAL(
      get_logger(),
      "takeoff_height_m must be > 0 (got %.3f); refusing to start",
      takeoff_height_m_);
    throw std::invalid_argument("takeoff_height_m must be > 0");
  }

  if (!use_relative_target_) {
    setpoint_x_ = target_x_;
    setpoint_y_ = target_y_;
    setpoint_z_ = target_z_;
  }

  offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", px4CommandQos());
  trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", px4CommandQos());
  vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", px4CommandQos());

  vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status_v4", px4SensorQos(),
    std::bind(&OffboardHoverNode::onVehicleStatus, this, std::placeholders::_1));
  vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry", px4SensorQos(),
    std::bind(&OffboardHoverNode::onVehicleOdometry, this, std::placeholders::_1));

  timer_ = create_wall_timer(50ms, std::bind(&OffboardHoverNode::timerCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "SITL hover node started (relative=%s takeoff_height=%.2f m, rel_offset_NED_xy=[%.2f, %.2f] m, abs NED [%.2f, %.2f, %.2f] yaw=%.2f, hover=%.1fs, prestream=%.1fs, pos_tol=%.2f m, reach_timeout=%.1fs)",
    use_relative_target_ ? "true" : "false", takeoff_height_m_,
    relative_offset_x_m_, relative_offset_y_m_,
    static_cast<double>(target_x_), static_cast<double>(target_y_),
    static_cast<double>(target_z_), static_cast<double>(target_yaw_),
    hover_duration_s_, prestream_duration_s_, position_tolerance_m_, target_reach_timeout_s_);
}

void OffboardHoverNode::onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  last_status_ = *msg;
  if (msg->system_id != 0) {
    target_system_ = msg->system_id;
  }
  if (msg->component_id != 0) {
    target_component_ = msg->component_id;
  }
}

void OffboardHoverNode::onVehicleOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  last_odometry_ = *msg;
}

void OffboardHoverNode::timerCallback()
{
  const rclcpp::Time t = now();
  const double in_state_s = elapsedSeconds(state_entered_, t);

  switch (state_) {
    case MissionState::kWaitForPx4:
      publishStreamingSetpoints();
      if (hasFreshStatus()) {
        transitionTo(MissionState::kPrestreamSetpoints, "vehicle_status received");
      } else if (in_state_s >= px4_ready_timeout_s_) {
        finishAbortedDisarmed("px4_ready_timeout_s expired with no VehicleStatus");
      }
      break;

    case MissionState::kPrestreamSetpoints:
      if (isArmed()) {
        enterRequestLand("armed during prestream");
        break;
      }
      if (use_relative_target_) {
        tryCaptureHome();
      }
      publishStreamingSetpoints();
      if (use_relative_target_ && !home_locked_) {
        if (in_state_s >= prestream_timeout_s_) {
          finishAbortedDisarmed("prestream_timeout_s expired with no home odometry lock");
        }
        break;
      }
      {
        const double stream_s = use_relative_target_ ?
          elapsedSeconds(home_locked_at_, t) : in_state_s;
        if (stream_s >= prestream_duration_s_) {
          transitionTo(MissionState::kRequestOffboard, "prestream complete");
        } else if (in_state_s >= prestream_timeout_s_) {
          finishAbortedDisarmed("prestream_timeout_s expired");
        }
      }
      break;

    case MissionState::kRequestOffboard:
      publishStreamingSetpoints();
      if (isOffboard()) {
        transitionTo(MissionState::kRequestArm, "nav_state OFFBOARD");
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

    case MissionState::kRequestArm:
      publishStreamingSetpoints();
      if (!isOffboard()) {
        if (isArmed()) {
          enterRequestLand("offboard lost while requesting arm");
        } else {
          transitionTo(MissionState::kRequestOffboard, "offboard lost before arm");
        }
      } else if (isArmed()) {
        transitionTo(MissionState::kHover, "arming_state ARMED");
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

    case MissionState::kHover:
      publishStreamingSetpoints();
      if (last_status_.has_value() && last_status_->failsafe) {
        if (isArmed()) {
          enterRequestLand("PX4 failsafe reported during hover");
        } else {
          finishAbortedDisarmed("PX4 failsafe reported while disarmed during hover");
        }
      } else if (!isOffboard() || !isArmed()) {
        if (isArmed()) {
          enterRequestLand("offboard or arming feedback lost during hover");
        } else {
          finishAbortedDisarmed("disarmed during hover");
        }
      } else {
        maybeLockTargetReached();
        if (target_reached_) {
          if (elapsedSeconds(target_reached_at_, t) >= hover_duration_s_) {
            enterRequestLand("hover_duration_s complete");
          }
        } else if (in_state_s >= target_reach_timeout_s_) {
          if (isArmed()) {
            enterRequestLand("target_reach_timeout_s expired");
          } else {
            finishAbortedDisarmed("target_reach_timeout_s expired while disarmed");
          }
        }
      }
      break;

    case MissionState::kRequestLand:
      if (isDisarmed()) {
        transitionTo(MissionState::kWaitDisarm, "already DISARMED");
      } else if (isAutoLand()) {
        transitionTo(MissionState::kWaitDisarm, "nav_state AUTO_LAND");
      } else if (landingTimedOut(t) && isArmed()) {
        transitionTo(MissionState::kFailedLanding, "landing_timeout_s expired while armed");
      } else if (maybeRetryCommand()) {
        requestLand();
      }
      break;

    case MissionState::kWaitDisarm:
      if (isDisarmed()) {
        transitionTo(MissionState::kDone, "arming_state DISARMED");
      } else if (landingTimedOut(t) && isArmed()) {
        transitionTo(MissionState::kFailedLanding, "landing_timeout_s expired while armed");
      } else if (!isAutoLand() && !isDisarmed() && maybeRetryCommand()) {
        requestLand();
      }
      break;

    case MissionState::kDone:
    case MissionState::kAbortedDisarmed:
    case MissionState::kFailedLanding:
      break;
  }
}

void OffboardHoverNode::publishStreamingSetpoints()
{
  if (!trajectoryTargetReady()) {
    return;
  }
  publishOffboardControlMode();
  publishTrajectorySetpoint();
}

void OffboardHoverNode::publishOffboardControlMode()
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

void OffboardHoverNode::publishTrajectorySetpoint()
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.timestamp = timestampUs();
  msg.position = {setpoint_x_, setpoint_y_, setpoint_z_};
  msg.velocity = {nan, nan, nan};
  msg.acceleration = {nan, nan, nan};
  msg.jerk = {nan, nan, nan};
  msg.yaw = target_yaw_;
  msg.yawspeed = nan;
  trajectory_setpoint_pub_->publish(msg);
}

void OffboardHoverNode::publishVehicleCommand(uint32_t command, float param1, float param2)
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

void OffboardHoverNode::requestOffboardMode()
{
  publishVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
  RCLCPP_INFO(get_logger(), "Sent VEHICLE_CMD_DO_SET_MODE (custom=1, offboard=6)");
}

void OffboardHoverNode::requestArm()
{
  publishVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    static_cast<float>(px4_msgs::msg::VehicleCommand::ARMING_ACTION_ARM),
    0.0f);
  RCLCPP_INFO(get_logger(), "Sent VEHICLE_CMD_COMPONENT_ARM_DISARM (arm)");
}

void OffboardHoverNode::requestLand()
{
  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0f, 0.0f);
  RCLCPP_INFO(get_logger(), "Sent VEHICLE_CMD_NAV_LAND");
}

void OffboardHoverNode::transitionTo(MissionState next, const std::string & reason)
{
  if (next == MissionState::kFailedLanding) {
    RCLCPP_ERROR(
      get_logger(), "STATE %s -> %s (%s)",
      stateName(state_), stateName(next), reason.c_str());
  } else if (next == MissionState::kAbortedDisarmed) {
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
  if (next == MissionState::kHover) {
    target_reached_ = false;
  }
}

void OffboardHoverNode::finishAbortedDisarmed(const std::string & reason)
{
  transitionTo(MissionState::kAbortedDisarmed, reason);
}

void OffboardHoverNode::enterRequestLand(const std::string & reason)
{
  landing_started_ = now();
  transitionTo(MissionState::kRequestLand, reason);
}

bool OffboardHoverNode::odometryPositionValid() const
{
  if (!last_odometry_.has_value()) {
    return false;
  }
  const auto & p = last_odometry_->position;
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

void OffboardHoverNode::maybeLockTargetReached()
{
  if (target_reached_ || !odometryPositionValid()) {
    return;
  }
  const auto & p = last_odometry_->position;
  const double dx = static_cast<double>(p[0]) - static_cast<double>(setpoint_x_);
  const double dy = static_cast<double>(p[1]) - static_cast<double>(setpoint_y_);
  const double dz = static_cast<double>(p[2]) - static_cast<double>(setpoint_z_);
  const double error_norm = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (error_norm > position_tolerance_m_) {
    return;
  }
  target_reached_ = true;
  target_reached_at_ = now();
  RCLCPP_INFO(
    get_logger(),
    "Target reached: error=%.4f m; starting hover timer",
    error_norm);
}

bool OffboardHoverNode::trajectoryTargetReady() const
{
  if (!use_relative_target_) {
    return true;
  }
  return home_locked_;
}

bool OffboardHoverNode::tryCaptureHome()
{
  if (!use_relative_target_ || home_locked_) {
    return home_locked_;
  }
  if (!isDisarmed() || !odometryPositionValid()) {
    return false;
  }
  home_x_ = last_odometry_->position[0];
  home_y_ = last_odometry_->position[1];
  home_z_ = last_odometry_->position[2];
  setpoint_x_ = static_cast<float>(static_cast<double>(home_x_) + relative_offset_x_m_);
  setpoint_y_ = static_cast<float>(static_cast<double>(home_y_) + relative_offset_y_m_);
  setpoint_z_ = static_cast<float>(static_cast<double>(home_z_) - takeoff_height_m_);
  home_locked_ = true;
  home_locked_at_ = now();
  RCLCPP_INFO(
    get_logger(),
    "Home locked: home NED [%.4f, %.4f, %.4f], offset NED [%.4f, %.4f] m (N,E), target NED [%.4f, %.4f, %.4f] (z = home_z - %.3f)",
    static_cast<double>(home_x_), static_cast<double>(home_y_), static_cast<double>(home_z_),
    relative_offset_x_m_, relative_offset_y_m_,
    static_cast<double>(setpoint_x_), static_cast<double>(setpoint_y_),
    static_cast<double>(setpoint_z_), takeoff_height_m_);
  return true;
}

bool OffboardHoverNode::hasFreshStatus() const
{
  return last_status_.has_value();
}

bool OffboardHoverNode::isArmed() const
{
  return last_status_.has_value() &&
    last_status_->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
}

bool OffboardHoverNode::isDisarmed() const
{
  return last_status_.has_value() &&
    last_status_->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
}

bool OffboardHoverNode::isOffboard() const
{
  return last_status_.has_value() &&
    last_status_->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

bool OffboardHoverNode::isAutoLand() const
{
  return last_status_.has_value() &&
    last_status_->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
}

bool OffboardHoverNode::landingTimedOut(const rclcpp::Time & t) const
{
  return elapsedSeconds(landing_started_, t) >= landing_timeout_s_;
}

uint64_t OffboardHoverNode::timestampUs() const
{
  return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
}

const char * OffboardHoverNode::stateName(MissionState state)
{
  switch (state) {
    case MissionState::kWaitForPx4:
      return "WAIT_FOR_PX4";
    case MissionState::kPrestreamSetpoints:
      return "PRESTREAM_SETPOINTS";
    case MissionState::kRequestOffboard:
      return "REQUEST_OFFBOARD";
    case MissionState::kRequestArm:
      return "REQUEST_ARM";
    case MissionState::kHover:
      return "HOVER";
    case MissionState::kRequestLand:
      return "REQUEST_LAND";
    case MissionState::kWaitDisarm:
      return "WAIT_DISARM";
    case MissionState::kDone:
      return "DONE";
    case MissionState::kAbortedDisarmed:
      return "ABORTED_DISARMED";
    case MissionState::kFailedLanding:
      return "FAILED_LANDING";
  }
  return "UNKNOWN";
}

bool OffboardHoverNode::maybeRetryCommand()
{
  if (!command_sent_in_state_) {
    return true;
  }
  return elapsedSeconds(last_command_sent_, now()) >= 1.0;
}

}  // namespace px4_interface

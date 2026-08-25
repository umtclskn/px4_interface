#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

namespace px4_interface
{

enum class MissionState : uint8_t
{
  kWaitForPx4,
  kPrestreamSetpoints,
  kRequestOffboard,
  kRequestArm,
  kHover,
  kRequestLand,
  kWaitDisarm,
  kDone,
  kAbortedDisarmed,
  kFailedLanding,
};

class OffboardHoverNode : public rclcpp::Node
{
public:
  OffboardHoverNode();

private:
  void timerCallback();
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void onVehicleOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  void publishOffboardControlMode();
  void publishTrajectorySetpoint();
  void publishVehicleCommand(uint32_t command, float param1, float param2);
  void publishStreamingSetpoints();

  bool tryCaptureHome();
  bool trajectoryTargetReady() const;
  bool odometryPositionValid() const;
  void maybeLockTargetReached();

  void requestOffboardMode();
  void requestArm();
  void requestLand();

  void transitionTo(MissionState next, const std::string & reason);
  void finishAbortedDisarmed(const std::string & reason);
  void enterRequestLand(const std::string & reason);

  bool hasFreshStatus() const;
  bool isArmed() const;
  bool isDisarmed() const;
  bool isOffboard() const;
  bool isAutoLand() const;
  bool landingTimedOut(const rclcpp::Time & t) const;

  uint64_t timestampUs() const;
  static const char * stateName(MissionState state);

  bool maybeRetryCommand();

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  MissionState state_{MissionState::kWaitForPx4};
  std::optional<px4_msgs::msg::VehicleStatus> last_status_;
  std::optional<px4_msgs::msg::VehicleOdometry> last_odometry_;

  rclcpp::Time state_entered_;
  rclcpp::Time last_command_sent_;
  rclcpp::Time landing_started_;
  rclcpp::Time home_locked_at_;
  rclcpp::Time target_reached_at_;
  bool command_sent_in_state_{false};
  bool use_relative_target_{true};
  bool home_locked_{false};
  bool target_reached_{false};

  uint8_t target_system_{1};
  uint8_t target_component_{1};

  double hover_duration_s_{10.0};
  double px4_ready_timeout_s_{30.0};
  double prestream_duration_s_{1.0};
  double prestream_timeout_s_{10.0};
  double offboard_timeout_s_{10.0};
  double arm_timeout_s_{10.0};
  double landing_timeout_s_{30.0};
  double takeoff_height_m_{1.0};
  double relative_offset_x_m_{0.0};
  double relative_offset_y_m_{0.0};
  double position_tolerance_m_{0.15};
  double target_reach_timeout_s_{45.0};
  float target_x_{0.0f};
  float target_y_{0.0f};
  float target_z_{-2.0f};
  float target_yaw_{0.0f};
  float home_x_{0.0f};
  float home_y_{0.0f};
  float home_z_{0.0f};
  float setpoint_x_{0.0f};
  float setpoint_y_{0.0f};
  float setpoint_z_{0.0f};
};

}  // namespace px4_interface

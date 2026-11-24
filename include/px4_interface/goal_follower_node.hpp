#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "px4_interface/frame_conversions.hpp"
#include "px4_interface/path_tracker.hpp"

namespace px4_interface
{

enum class GoalFollowerState : uint8_t
{
  kWaitForPx4,
  kWaitForAlignment,
  kPrestreamSetpoints,
  kRequestOffboard,
  kRequestArm,
  kTakeoff,
  kFollow,
  kHold,
  kRequestLand,
  kWaitDisarm,
  kDone,
  kAbortedDisarmed,
  kFailedLanding,
};

class GoalFollowerNode : public rclcpp::Node
{
public:
  explicit GoalFollowerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  GoalFollowerState state() const noexcept {return state_;}

private:
  void timerCallback();
  void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void onVehicleOdometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void onPath(const nav_msgs::msg::Path::SharedPtr msg);
  void onLand(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void publishOffboardControlMode();
  bool publishTrajectorySetpoint(const Vec3 & target_ned);
  void publishVehicleCommand(uint32_t command, float param1, float param2);
  bool publishStreamingSetpoints(const Vec3 & target_ned);

  bool tryCaptureAlignment();
  bool odometryNedValid() const;
  Vec3 vehicleNed() const;
  Vec3 takeoffSetpointNed() const;
  bool withinTolerance(const Vec3 & target_ned) const;
  bool pathUsable(const rclcpp::Time & t) const;
  void enterHold(const std::string & reason);
  void enterFollow(const std::string & reason);
  void updateFollowHold(const rclcpp::Time & t);

  void requestOffboardMode();
  void requestArm();
  void requestLand();

  void transitionTo(GoalFollowerState next, const std::string & reason);
  void finishAbortedDisarmed(const std::string & reason);
  void enterRequestLand(const std::string & reason);

  bool hasFreshStatus() const;
  bool isArmed() const;
  bool isDisarmed() const;
  bool isOffboard() const;
  bool isAutoLand() const;
  bool landingTimedOut(const rclcpp::Time & t) const;
  bool maybeRetryCommand();

  uint64_t timestampUs() const;
  static const char * stateName(GoalFollowerState state);
  static const char * pathRejectionName(PathRejection rejection);

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
    offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
    trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
    vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
    vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
    vehicle_odometry_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  GoalFollowerState state_{GoalFollowerState::kWaitForPx4};
  std::optional<px4_msgs::msg::VehicleStatus> last_status_;
  std::optional<px4_msgs::msg::VehicleOdometry> last_odometry_;

  rclcpp::Time state_entered_;
  rclcpp::Time last_command_sent_;
  rclcpp::Time landing_started_;
  rclcpp::Time last_path_stamp_;
  bool command_sent_in_state_{false};
  bool land_requested_{false};
  bool alignment_locked_{false};
  bool hold_latched_{false};

  WorldNedAlignment alignment_{};
  PathTracker path_tracker_;
  PathLimits path_limits_{};
  Vec3 hold_ned_{};
  Vec3 setpoint_ned_{};
  float hold_yaw_ned_{0.0f};

  uint8_t target_system_{1};
  uint8_t target_component_{1};
  std::size_t path_reject_count_{0};

  std::string world_frame_{"world"};
  std::string body_frame_{"body"};
  double world_yaw_offset_rad_{0.0};
  double takeoff_height_m_{1.5};
  double lookahead_m_{1.0};
  double position_tolerance_m_{0.2};
  double path_timeout_s_{3.0};
  std::string yaw_mode_{"hold"};
  double px4_ready_timeout_s_{30.0};
  double prestream_duration_s_{1.0};
  double prestream_timeout_s_{10.0};
  double offboard_timeout_s_{10.0};
  double arm_timeout_s_{10.0};
  double landing_timeout_s_{30.0};
};

}  // namespace px4_interface

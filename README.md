# px4_interface

SITL-only ROS 2 Jazzy node that streams PX4 offboard position setpoints, requests OFFBOARD + arm, hovers in NED, then lands. Independent of graph SLAM, Isaac Sim, and Pegasus packages.

## Phase 1 Status

Phase 1 **SITL acceptance is complete** (2026-08-25) on ROS 2 Jazzy + PX4 SITL + Pegasus in NVIDIA Isaac Sim + Micro XRCE-DDS Agent (UDP 8888). Validation is **SITL-only**; this package does not provide real-flight, VIO, LiDAR, mapping, planning, or exploration support. Recorded tests, pass criteria, and limits: [Phase 1 Acceptance Record](PHASE1_ACCEPTANCE.md).

## Defaults (launch with no extra args)

| Parameter | Default | Meaning |
|---|---|---|
| `use_relative_target` | `true` | Lock home from live `VehicleOdometry` while disarmed in PRESTREAM; hold that setpoint for the flight |
| `takeoff_height_m` | `1.0` | Relative climb in metres (must be `> 0`). NED: `setpoint_z = home_z - takeoff_height_m` |
| `relative_offset_x_m` | `0.0` | Relative mode only: NED **North** offset from home (m). Negative = South. Unused if `use_relative_target:=false` |
| `relative_offset_y_m` | `0.0` | Relative mode only: NED **East** offset from home (m). Negative = West. Unused if `use_relative_target:=false` |
| `hover_duration_s` | `10.0` | Hover time after 3D position error first drops to `position_tolerance_m` |
| `position_tolerance_m` | `0.15` | 3D `||odom − setpoint||` that starts the hover timer (must be `> 0`) |
| `target_reach_timeout_s` | `45.0` | Max time in HOVER to reach the setpoint (must be `> 0`); then LAND if armed |
| `px4_ready_timeout_s` | `30.0` | Abort if no `VehicleStatus` arrives |
| `prestream_duration_s` | `1.0` | Minimum heartbeat/setpoint stream before mode switch (relative mode: measured after home lock) |
| `prestream_timeout_s` | `10.0` | Max time in prestream (no home lock → `ABORTED_DISARMED`, no LAND) |
| `offboard_timeout_s` | `10.0` | Max wait for `NAVIGATION_STATE_OFFBOARD` |
| `arm_timeout_s` | `10.0` | Max wait for `ARMING_STATE_ARMED` |
| `landing_timeout_s` | `30.0` | Max wait for land/disarm |
| `target_x` `target_y` `target_z` `target_yaw` | `0`, `0`, `-2`, `0` | Used only if `use_relative_target:=false`. NED hover (`z` down) |

## Relative vs absolute setpoint

- `use_relative_target:=false`: `TrajectorySetpoint.position = {target_x, target_y, target_z}` from parameters (absolute local NED).
- `use_relative_target:=true` (default): no `TrajectorySetpoint` / `OffboardControlMode` until home is locked. Home is captured **once** in `PRESTREAM_SETPOINTS` from a live `/fmu/out/vehicle_odometry` sample while **disarmed**. Formula: `{home_x + relative_offset_x_m, home_y + relative_offset_y_m, home_z - takeoff_height_m}`. Yaw stays `target_yaw`. Home and setpoint are never updated after lock (including after arm).
- `WAIT_FOR_PX4` in relative mode publishes **neither** stream topic (avoids an uninitialized or absolute `{0,0,-2}` fallback). After status arrives, PRESTREAM waits for odometry.

## Hover timer vs climb

HOVER still starts at ARMED + OFFBOARD, but `hover_duration_s` does **not**. The node keeps streaming the setpoint until `sqrt(dx^2+dy^2+dz^2) <= position_tolerance_m` on a valid odometry sample, logs `Target reached: error=... m; starting hover timer` once, then counts `hover_duration_s`. Invalid/missing odometry does not count as reached. If that never happens within `target_reach_timeout_s` of HOVER entry, armed vehicles get `REQUEST_LAND` (`target_reach_timeout_s expired`). Failsafe / offboard-loss / disarm still run first.

## Run (after PX4 SITL + MicroXRCEAgent)

```bash
cd  /{ros2_ws}/ # your ros 2 workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch px4_interface sitl_hover.launch.py \
  relative_offset_x_m:=1.0 \
  relative_offset_y_m:=0.0 \
  takeoff_height_m:=1.0 \
  position_tolerance_m:=0.20 \
  target_reach_timeout_s:=60.0 \
  hover_duration_s:=10.0
# vertical-only relative: omit offsets (defaults 0,0)
# ros2 launch px4_interface sitl_hover.launch.py takeoff_height_m:=1.0 position_tolerance_m:=0.15
# absolute NED: ros2 launch px4_interface sitl_hover.launch.py use_relative_target:=false target_z:=-2.0
```

## Contract

- In: `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command`
- Out: `/fmu/out/vehicle_status_v4`, `/fmu/out/vehicle_odometry`
- Success is `DONE` after `ARMING_STATE_DISARMED` following a land.
- `ABORTED_DISARMED` and `FAILED_LANDING` are failure terminals (no force-disarm).

## Velocity keyboard teleop

SITL keyboard velocity teleop (do **not** run alongside `offboard_hover_node`):

```bash
ros2 run px4_interface velocity_teleop_bridge
# separate raw-TTY terminal:
ros2 run px4_interface velocity_teleop_keyboard
```

Topics: `/offboard_velocity_cmd`, `/arm_message` → `/fmu/in/*`.

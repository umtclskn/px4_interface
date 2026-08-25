# Phase 1 Acceptance Record

## Scope

Phase 1 closes SITL acceptance of the standalone ROS 2 Jazzy package `px4_interface` (`offboard_hover_node`): PX4 offboard position control as a **relative** (and optional absolute) NED setpoint primitive, with a target-reached hover timer and a safe land/abort state machine.

This record documents the validated environment, interface, safety policy, coordinate convention, key parameters, reproducible tests, measured results, pass criteria, known limits, and the handoff into Phase 2. It does **not** claim real-aircraft, VIO, LiDAR, mapping, planning, or exploration capability.

**Date:** 2026-08-25

## Validated Environment

- ROS 2 Jazzy
- PX4 SITL
- Pegasus Simulator running in NVIDIA Isaac Sim
- Micro XRCE-DDS Agent over UDP port 8888

## Package Interface

- Package: `px4_interface`
- Executable: `offboard_hover_node`

PX4 ROS 2 topics:

| Direction | Topic |
|---|---|
| Subscribe | `/fmu/out/vehicle_status_v4` |
| Subscribe | `/fmu/out/vehicle_odometry` |
| Publish | `/fmu/in/offboard_control_mode` |
| Publish | `/fmu/in/trajectory_setpoint` |
| Publish | `/fmu/in/vehicle_command` |

**QoS** (all listed subscriptions and publishers): KeepLast(1), BEST_EFFORT, VOLATILE.

## Safety and State Machine

- 20 Hz position-mode `OffboardControlMode` and `TrajectorySetpoint` streaming before OFFBOARD, while requesting OFFBOARD, ARM, and during HOVER.
- `VehicleCommand` retries are limited to at most 1 Hz.
- Mode and arm success come from `VehicleStatus` feedback, not from command publish.
- OFFBOARD stream and trajectory setpoint publication **stop** on `REQUEST_LAND`.
- Landing uses PX4 `NAV_LAND`.
- Landing completion is confirmed by `VehicleStatus` disarmed feedback.
- Failsafe, OFFBOARD loss, or unexpected disarm during HOVER follow the existing safe landing/abort policy (`REQUEST_LAND` or `ABORTED_DISARMED` as implemented).
- Only one offboard controller may publish to `/fmu/in/*` topics at a time.

Relative-target streaming starts only after a relative target is ready (home locked). Terminal success is `DONE`; failure terminals are `ABORTED_DISARMED` and `FAILED_LANDING` (no force-disarm).

## Coordinate Convention

PX4 local NED:

- X positive = North
- Y positive = East
- Z positive = Down
- Negative Z is upward motion

## Key Parameters

Relative-target (default `use_relative_target=true`):

- Live home is captured **once** from `VehicleOdometry` while **disarmed** in `PRESTREAM_SETPOINTS`.
- Setpoint (fixed for the rest of the flight):

```text
setpoint_x = home_x + relative_offset_x_m
setpoint_y = home_y + relative_offset_y_m
setpoint_z = home_z - takeoff_height_m
```

Target-reached:

- 3D Euclidean position error between `VehicleOdometry` and the active setpoint.
- `hover_duration_s` starts only after the error is within `position_tolerance_m`.
- `target_reach_timeout_s` triggers the existing safe terminal behavior if the target is not reached.

## Reproducible Acceptance Tests

Workspace overlay (operator-run; not executed as part of this documentation task):

```bash
cd ~/portfolio_ws/drone_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Preconditions: PX4 SITL, Pegasus/Isaac Sim, MicroXRCEAgent `udp4 -p 8888`, **one** `offboard_hover_node`, vehicle disarmed before launch.

### Test A — relative vertical target

```bash
ros2 launch px4_interface sitl_hover.launch.py \
  use_relative_target:=true \
  takeoff_height_m:=1.0 \
  position_tolerance_m:=0.15 \
  target_reach_timeout_s:=45.0 \
  hover_duration_s:=10.0
```

### Test B — relative 3D waypoint

```bash
ros2 launch px4_interface sitl_hover.launch.py \
  relative_offset_x_m:=1.0 \
  relative_offset_y_m:=0.0 \
  takeoff_height_m:=1.0 \
  position_tolerance_m:=0.20 \
  target_reach_timeout_s:=60.0 \
  hover_duration_s:=10.0
```

## Verified Results

### Test A

- `use_relative_target=true`, `takeoff_height_m=1.0`, `position_tolerance_m=0.15`, `target_reach_timeout_s=45.0`, `hover_duration_s=10.0`
- Verified target-reached error: **0.1496 m**
- Verified terminal sequence: `REQUEST_OFFBOARD` → `REQUEST_ARM` → `HOVER` → Target reached → `REQUEST_LAND` → `WAIT_DISARM` → `DONE`

### Test B

- Home NED: `[0.0028, -0.0102, 0.0032]`
- Target NED: `[1.0028, -0.0102, -0.9968]`
- Target reached error: **0.1990 m**
- Disarmed landing position: `[0.9617, 0.0593, 0.0051]`
- Observed displacement from home:
  - North/X: approximately **+0.959 m**
  - East/Y: approximately **+0.069 m**
  - Down/Z after landing: approximately **+0.002 m** relative to home
- Calculated horizontal landing error from target: approximately **0.081 m**
- Verified terminal sequence: `WAIT_FOR_PX4` → `PRESTREAM_SETPOINTS` → `REQUEST_OFFBOARD` → `REQUEST_ARM` → `HOVER` → Target reached → `REQUEST_LAND` → `WAIT_DISARM` → `DONE`

## Pass Criteria

Phase 1 SITL acceptance **passes** when, on the validated stack above:

1. Relative home lock and fixed setpoint match the documented formula.
2. Target-reached error is within the configured `position_tolerance_m` before the hover timer.
3. The hover/land sequence ends in `DONE` after disarmed `VehicleStatus`.
4. Tests A and B match the verified results recorded in this file.

Tests A and B as recorded on **2026-08-25** meet these criteria.

## Known Scope Limits

- Validation is PX4 SITL-only through Pegasus/Isaac Sim.
- No real-aircraft validation has been performed.
- No VIO or SLAM integration is validated.
- No 3D LiDAR integration, occupancy mapping, frontier/NBV exploration, global planner, local obstacle avoidance, multi-waypoint mission, dynamic-obstacle handling, or real-time safety validation is included.
- This package provides a position-target primitive; it is not a full trajectory controller.
- Yaw is held at the configured target yaw; yaw tracking is not a Phase 1 acceptance objective.
- Only one offboard controller may publish to `/fmu/in/*` simultaneously.

## Phase 2 Handoff

Phase 2 begins the architecture and interface contract for an autopilot-independent `drone_exploration` ROS 2 package. The future exploration core will accept generic state, TF, mapping/map-query, and optional sensor inputs. It will output generic pose/path/trajectory goals. PX4 integration will remain an adapter concern, rather than a dependency of the generic exploration core.

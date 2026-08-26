# Phase 1 — Step 9: Relative 3D Waypoint SITL Test Guide

This guide validates a single relative 3D waypoint using the `px4_interface` package.

The test command asks the vehicle to move relative to the live home position captured by `VehicleOdometry`:

```text
North (NED X): +1.0 m
East  (NED Y): +0.0 m
Up    (NED Z): -1.0 m
```

The node must reach the target, remain within tolerance for 10 seconds, request a PX4 land, wait for disarm, and finish with `DONE`.

## Coordinate convention

PX4 local position setpoints use the NED frame:

- `X+`: North
- `Y+`: East
- `Z+`: Down
- Moving up means a more negative `Z` value

In relative-target mode, the expected target is:

```text
setpoint_x = home_x + relative_offset_x_m
setpoint_y = home_y + relative_offset_y_m
setpoint_z = home_z - takeoff_height_m
```

For this test:

```text
relative_offset_x_m = 1.0
relative_offset_y_m = 0.0
takeoff_height_m    = 1.0
```

## Prerequisites

Before starting:

- The Step 9 `px4_interface` changes must be built successfully.
- Isaac Sim, Pegasus Simulator, PX4 SITL, and Micro XRCE-DDS Agent must be available.
- Only one `offboard_hover_node` may publish to `/fmu/in/*`.
- The drone must begin on the ground, disarmed, with motors stopped.
- Do not run the old Python `px4_offboard` mission or another offboard controller during this test.

## Terminal 1 — Isaac Sim and Pegasus

Open a new terminal and start Isaac Sim with the Pegasus extension folder.

```bash
export ISAACSIM_PATH="/home/umut/Desktop/isaac-sim"
export ISAACSIM="$ISAACSIM_PATH/isaac-sim.sh"
export PEGASUS_REPO="$HOME/PegasusSimulator"

"$ISAACSIM" --ext-folder "$PEGASUS_REPO/extensions"
```

After the Isaac Sim GUI opens:

1. Open the Pegasus Simulator extension or panel.
2. Select the PX4 backend.
3. Enable PX4 auto-launch if that is the established project configuration.
4. Confirm the PX4 path is:

```text
/home/umut/PX4-Autopilot
```

5. Load the default environment using **Load Scene**.
6. Select the Iris vehicle.
7. Set Vehicle ID to `0`.
8. Set the initial spawn position to:

```text
x = 0.0
y = 0.0
z = 0.5
```

9. Select **Load Vehicle**.
10. Press **Play** once to start the simulation.

Leave Isaac Sim running. At this point, the drone should be on the ground, disarmed, and not spinning its motors.

Do not start the ROS controller in this terminal.

## Terminal 2 — Micro XRCE-DDS Agent

Open a second terminal and start the agent:

```bash
source /opt/ros/jazzy/setup.bash
MicroXRCEAgent udp4 -p 8888
```

Leave this terminal running.

Expected result: the agent shows an established PX4 client/session after PX4 SITL is running.

## Terminal 3 — ROS environment and preflight check

Open a third terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/{ros2_ws}/install/setup.bash
```

Verify that no old controller process is still running:

```bash
pgrep -af offboard_hover_node
```

Expected result before the flight launch: no output.

If an old `offboard_hover_node` is listed, do not start another one. Stop the old controller from its original launch terminal with `Ctrl+C`, then run the check again.

Check the ROS graph before launch:

```bash
ros2 node list | grep -E 'offboard|px4_interface' || true
```

Expected result before the flight launch: no active `offboard_hover_node`.

Keep this terminal available for publisher verification during the flight.

## Terminal 4 — Odometry observation

Open a fourth terminal before launching the controller:

```bash
source /opt/ros/jazzy/setup.bash
source ~/{ros2_ws}/install/setup.bash

ros2 topic echo /fmu/out/vehicle_odometry --field position \
  --qos-reliability best_effort \
  --qos-durability volatile
```

Leave this command running through takeoff, waypoint tracking, hover, landing, and disarm.

Expected position pattern:

```text
Ground:      [home_x, home_y, home_z]
Movement:    X approaches home_x + 1.0
             Y remains near home_y
             Z approaches home_z - 1.0
Hover:       approximately [home_x + 1.0, home_y, home_z - 1.0]
Landing:     Z returns near home_z
```

Do not stop the command until the drone has landed and remained on the ground for a few seconds.

## Terminal 5 — Vehicle status observation

Open a fifth terminal before launching the controller:

```bash
source /opt/ros/jazzy/setup.bash
source ~/{ros2_ws}/install/setup.bash

ros2 topic echo /fmu/out/vehicle_status_v4 \
  --qos-reliability best_effort \
  --qos-durability volatile
```

Watch these fields:

```text
arming_state
nav_state
failsafe
```

Expected high-level sequence:

```text
DISARMED
→ ARMED + OFFBOARD
→ AUTO_LAND
→ DISARMED
```

If `failsafe: true` appears, keep the controller and PX4 logs for diagnosis.

## Terminal 6 — Controller launch

Open a sixth terminal and source the ROS overlays:

```bash
source /opt/ros/jazzy/setup.bash
source ~/{ros2_ws}/install/setup.bash
```

Launch exactly one controller:

```bash
ros2 launch px4_interface sitl_hover.launch.py \
  relative_offset_x_m:=1.0 \
  relative_offset_y_m:=0.0 \
  takeoff_height_m:=1.0 \
  position_tolerance_m:=0.20 \
  target_reach_timeout_s:=60.0 \
  hover_duration_s:=10.0
```

Expected controller log sequence:

```text
STATE WAIT_FOR_PX4 -> PRESTREAM_SETPOINTS
Home locked from live VehicleOdometry NED [...]
STATE PRESTREAM_SETPOINTS -> REQUEST_OFFBOARD
STATE REQUEST_OFFBOARD -> REQUEST_ARM
STATE REQUEST_ARM -> HOVER
Target reached: error=... m; starting hover timer
STATE HOVER -> REQUEST_LAND (hover_duration_s complete)
STATE REQUEST_LAND -> WAIT_DISARM (nav_state AUTO_LAND)
STATE WAIT_DISARM -> DONE (arming_state DISARMED)
```

Do not press `Ctrl+C` after landing begins. Wait for this final line:

```text
STATE WAIT_DISARM -> DONE (arming_state DISARMED)
```

Only then may you stop the controller with `Ctrl+C`.

## Publisher verification

While Terminal 6 is running, return to Terminal 3 and run:

```bash
ros2 topic info /fmu/in/trajectory_setpoint -v
```

Expected result:

```text
Publisher count: 1
Node name: offboard_hover_node
```

If there is more than one publisher, stop the test and do not interpret the odometry result. Multiple controllers can conflict on `/fmu/in/trajectory_setpoint`, `/fmu/in/offboard_control_mode`, and `/fmu/in/vehicle_command`.

## Success criteria

The Step 9 test passes when all of the following are true:

- The controller logs a live `Home locked` message.
- The logged target equals `home + [1.0, 0.0, -1.0]` in NED coordinates.
- There is exactly one `TrajectorySetpoint` publisher during the flight.
- Odometry shows X moving approximately 1 m positive from home.
- Odometry shows Y remaining close to home Y.
- Odometry shows Z moving approximately 1 m negative from home Z.
- The node logs `Target reached`.
- The node completes the configured 10-second hover after target reach.
- The node reaches `WAIT_DISARM -> DONE` after PX4 AUTO_LAND and disarm.
- `failsafe` remains false during normal tracking.

## Evidence to save

After the test, keep or share:

- Full controller logs containing `Home locked`, state transitions, target reached, and `DONE`.
- Odometry values at ground, closest approach to target, and after landing.
- Output of `ros2 topic info /fmu/in/trajectory_setpoint -v` during flight.
- Any PX4 or Isaac/Pegasus warning, failsafe, or mode-change message.

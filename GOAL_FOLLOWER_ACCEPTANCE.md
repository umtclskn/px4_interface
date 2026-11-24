# Goal follower acceptance (PX4 SITL + Isaac Sim)

Fill the **Result** cells during a manual SITL run. Copy measured numbers into
`docs/exploration_plan/PROGRESS.md`.

**Environment:** ROS 2 Jazzy · PX4 SITL · Micro XRCE-DDS · Isaac Sim + Pegasus  
**Date:** _______________ · **Operator:** _______________

| # | Check | Result | Notes / numbers |
|---|---|---|---|
| 1 | Take-off to `takeoff_height_m`, hold 30 s, land via `~/land` | ☐ pass / ☐ fail | height error ___ m |
| 2a | `publish_test_path.py --axis x --length 2` moves along world +x | ☐ pass / ☐ fail | |
| 2b | `publish_test_path.py --axis y --length 2` moves along world +y | ☐ pass / ☐ fail | If mirrored/swapped: stop; check `world_yaw_offset_rad` / ENU |
| 3 | Stop path publisher → `HOLD` within `path_timeout_s`, no drift | ☐ pass / ☐ fail | hold error ___ m |
| 4 | Path start 10 m away → `kStartTooFar`, stays `HOLD` | ☐ pass / ☐ fail | |
| 5 | Mean speed on 5 m straight path at `lookahead_m=1.0` | ☐ measured | ___ m/s |
| 6 | EKF2 vs GT (TF vs odom via alignment), 2 min hover | ☐ measured | ___ m |
| 7 | `ros2 topic info /fmu/in/trajectory_setpoint -v` → one publisher | ☐ pass / ☐ fail | |

## Commands (reference)

```bash
ros2 launch px4_interface goal_follower.launch.py
ros2 run px4_interface publish_test_path.py --axis x --length 2
ros2 service call /goal_follower_node/land std_srvs/srv/Trigger
```

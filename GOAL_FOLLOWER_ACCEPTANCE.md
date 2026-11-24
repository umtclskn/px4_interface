# Goal follower acceptance (PX4 SITL + Isaac Sim)

Fill the **Result** cells during a manual SITL run. Copy measured numbers into
`docs/exploration_plan/PROGRESS.md`.

**Environment:** ROS 2 Jazzy · PX4 SITL · Micro XRCE-DDS · Isaac Sim + Pegasus  
**Date:** 2026-11-24 · **Operator:** umut (+ Auto assist)  
**Bags:**  
- item 6 hover: `~/bags/sitl_t12_20261124_200844` (window wall **20:12:50–20:14:50 +01:00**)  
- items 3/4/8 + partial: `~/bags/sitl_t12_20261124_202509`  
- item 5 speed: `~/bags/sitl_t12_20261124_204242` (motion **20:43:40–20:44:05 +01:00**)  
**Note:** `publish_test_path.py` re-anchors +length from *current* pose every 1 s — kill early (~target displacement) or it chases into walls.  
**PX4 params:** `COM_OF_LOSS_T` / `COM_OBL_RC_ACT` — not recorded (console read skipped).  
**Extra (T12b):** empty `/path` → log `empty path accepted -> hold`; `frame_id=map` → `path rejected: frame_id='map' != world_frame='world'`.

| # | Check | Result | Notes / numbers |
|---|---|---|---|
| 1 | Take-off to `takeoff_height_m`, hold 30 s, land via `~/land` | ☑ pass / ☐ fail | height error **~0.15 m** (e.g. z≈1.35 vs 1.5); Isaac hover stable [VISUAL]. Land via `/goal_follower_node/land` → DONE (item-5 session). |
| 2a | `publish_test_path.py --axis x --length 2` moves along world +x | ☑ pass / ☐ fail | cut near x≈2.17, y≈0; Isaac red (+x) [VISUAL]. |
| 2b | `publish_test_path.py --axis y --length 2` moves along world +y | ☑ pass / ☐ fail | TF after ~`(3.0, 2.95, …)` (+y); not mirrored/swapped. Warehouse wall risk if publisher left running. |
| 3 | Stop path publisher → `HOLD` within `path_timeout_s`, no drift | ☑ pass / ☐ fail | HOLD @ path_stale; 10 s hold error **0.241 m**. |
| 4 | Path start 10 m away → `kStartTooFar`, stays `HOLD` | ☑ pass / ☐ fail | log `path rejected: start_too_far`; stayed HOLD. |
| 5 | Mean speed on 5 m straight path at `lookahead_m=1.0` | ☑ measured | **0.236 m/s** (path 5.35 m / 22.7 s, v>0.1); bag `…204242`; kill at dx≈4.84 m. |
| 6 | EKF2 vs GT (TF vs odom via alignment), 2 min hover | ☑ measured | relative TF vs NED→ENU odom: **mean 0.014 m, max 0.028 m**; bag `…200844`. |
| 7 | `ros2 topic info /fmu/in/trajectory_setpoint -v` → one publisher | ☑ pass / ☐ fail | exactly 1 publisher: `goal_follower_node`. |

## Commands (reference)

```bash
ros2 launch px4_interface goal_follower.launch.py
ros2 run px4_interface publish_test_path.py --axis x --length 2
ros2 service call /goal_follower_node/land std_srvs/srv/Trigger
```

## Bag analysis helper

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/docs/exploration_plan/tools/analyze_goal_follower_bag.py \
  --bag ~/bags/sitl_t12_YYYYMMDD_HHMMSS --item 5 --t0 <ros_s> --t1 <ros_s>
```

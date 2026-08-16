# Current Architecture (inherited baseline)

Source: `warehouse-amr-ros2` @ commit `4e8aea5`, package
`ros2-package/amr_ros_dg`, imported into this repo as `src/amr_ros_dg`.
This document is the output of the Phase 0 repository audit.

## Component map

```
launch/fleet.launch.py
  ├─ gz_sim (Gazebo Harmonic, world=worlds/warehouse_fleet_world.sdf)
  ├─ per robot i in 1..num_robots, namespace "robot{i}":
  │    ├─ spawn (ros_gz_sim create -name robot{i})
  │    ├─ joint_state_broadcaster + diff_drive_controller spawners
  │    │    (chained off spawn via OnProcessExit)
  │    ├─ async_slam_toolbox_node (LifecycleNode, explicit
  │    │    configure→activate via EmitEvent/RegisterEventHandler)
  │    └─ nav2_bringup/navigation_launch.py, PushRosNamespace(robot{i}),
  │         nav2_params.yaml rewritten per-namespace
  │         (no map_server — SLAM supplies the map)
  └─ fleet_manager_node (single, non-namespaced), started ~20s after
       bringup so all per-robot Nav2 stacks are up first
```

Robot spawns are staggered 5.0 + 0.5*(i-1) s apart. Each robot's TF is
disambiguated via explicit `/tf → /robot{i}/tf` remaps (frame prefixing
is disabled in the controller config), not `frame_id` prefixing.

## Coordination today (single mode: "Mode A" centralized)

`scripts/fleet_manager.py` is a single node, not namespaced, running a
`MultiThreadedExecutor` so all robots' per-robot callbacks execute
concurrently in one process. Per robot it runs a state machine:

```
IDLE → NAVIGATING → PAUSED (2s, PICKUP_DROPOFF_PAUSE_SEC) → IDLE → ...
```

On entering IDLE it claims a random free station on the robot's current
side (`_claim_random_station`, `random.choice(free)`), sends a
`nav2_msgs/action/NavigateToPose` goal to `/{robot_ns}/navigate_to_pose`
via an `ActionClient`, and on arrival releases the station and flips to
the opposite side.

Station reservation is **not** a ROS-level protocol — it is a single
in-process `dict[station_name -> "claimed"]` guarded by a
`threading.Lock()`, because one process already owns the whole fleet's
scheduling. This will not survive decomposing the fleet manager into
independent per-robot agents (Mode C/D/E) — it needs to become a real
message-based protocol at that point.

There is no inter-robot collision avoidance beyond each robot's own
Nav2 local costmap / collision monitor consuming its own forward lidar.

## Robot health / state

None exists. No battery, mechanical health, sensor-confidence, or
agent-confidence model. This is the primary Phase 2 gap.

## Interfaces

No custom `.msg/.srv/.action` — only stock `geometry_msgs`,
`sensor_msgs`, `nav2_msgs`, `lifecycle_msgs`, `action_msgs`. Custom
agent-communication messages (§9 of the research spec) do not exist yet.

## Web dashboard

`scripts/web_dashboard.py` (Flask, plain HTTP polling, no websockets) +
`web/dashboard.html`: per-robot camera/lidar/ultrasonic/odom telemetry
viewer, plus a 10Hz teleop drive control with a 0.5s `cmd_vel` watchdog.
Reads `ROBOT_LIST` env var to know which robots to show.

## Testing

No `tests/` directory, no CI. "Test" files present
(`config/test_no_casters_controllers.yaml`,
`urdf/test_*.urdf.xacro`) are manual debugging variants, not automated
tests. `TROUBLESHOOTING.md` documents manual `ros2 topic hz` / `ros2
control list_controllers`-style checks as the de facto validation
method.

## Gap summary against the research spec

| Spec requirement | Status |
|---|---|
| Robot health/energy/confidence state (§5) | Missing |
| Fault injection framework (§6) | Missing |
| Coordination Modes B–E (§7) | Missing (only A exists) |
| Constrained agent action API + validators (§8) | Missing |
| Structured ROS messages for agent comms (§9) | Missing |
| Memory model (§10) | Missing |
| Experiment logging standard (§14) | Missing — this repo's Phase 1 |
| Deterministic seeding (§15) | Missing — this repo's Phase 1 |
| Automated tests | Missing |

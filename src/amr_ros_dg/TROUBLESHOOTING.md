# Troubleshooting / lessons learned — multi-robot fleet simulation

Every real bug hit while building `fleet.launch.py` (multi-robot SLAM+Nav2), in the order
they were found, with the actual symptom, root cause, and fix. Written so the next
debugging session checks this list FIRST instead of re-discovering the same bugs from
scratch — several of these cost hours because the symptom pointed somewhere misleading.

Read top to bottom the first time; after that, use the symptom column to jump straight
to a fix.

## Quick symptom index

| Symptom | Jump to |
|---|---|
| Only 1 robot spawns out of N | [#1](#1-only-1-robot-spawns--double-namespaced-nodes) |
| `controller_manager` hangs on "Waiting for data on 'robot_description'" | [#2](#2-controller_manager-never-comes-up-in-fleet-mode) |
| `SIGABRT` / "No critics defined for FollowPath" | [#3](#3-controller_server-crashes-no-critics-defined) |
| `The 'type' param was not defined for 'joint_state_broadcaster'` | [#4](#4-controller-yaml-silently-not-applied-once-namespaced) |
| `SIGABRT` in `collision_monitor` on startup | [#5](#5-collision_monitor-crashes-on-empty-polygon-list) |
| `"Charging dock plugins not given!"` | [#6](#6-docking_server-aborts-bringup) |
| Nav2 goals always "Action server is inactive" | [#7](#7-bt_navigator-goals-permanently-rejected) |
| `/tf`, `/tf_static` show up unnamespaced despite remap args | [#8](#8-tftf_static-never-actually-namespaced) |
| `ros2 topic list` / `ros2 node list` empty or stale | [#9](#9-ros2-daemon-staleness-in-wsl2) |
| "Timed out waiting for transform from base_link to odom" | [#10](#10-local-costmap-cant-find-odom-frame) |
| "Timed out waiting for transform from base_link to map" | [#11](#11-global-costmap-cant-find-map-frame) |
| slam_toolbox active but `/map` never appears | [#12](#12-slam_toolbox-never-publishes-map) |
| slam_toolbox: "Message Filter dropping message ... queue is full" | [#13](#13-lidar-frame_id-mismatch-in-slam_toolbox) |
| "Goal Coordinates ... was outside bounds" | [#14](#14-planner-goal-outside-costmap-bounds) |
| Nav2 says success/fail but wheels never move | [#15](#15-cmd_vel-published-to-a-dead-end-topic) |
| Same topic name, Nav2 publishes but controller never receives it | [#16](#16-twist-vs-twiststamped-type-mismatch) |
| fleet_manager retries once then silently stops forever | [#17](#17-create_timer-is-periodic-not-one-shot) |
| Assign→reject loop, hundreds/sec, robot never gets a real goal | [#18](#18-zero-delay-retry-loop-on-goal-rejection) |
| Robot completely frozen, odom never changes, no errors | [#19](#19-lidar-collision-geometry-embedded-in-the-chassis) / [#20](#20-lidar-self-detection-with-a-360deg-sweep) |
| Spawns fail at `num_robots` >= ~10: `Failed to acquire lock` | [#21](#21-controller-spawner-failures-under-load-at-scale) |
| Everything "works" but robots crawl / barely move | [#22](#22-global-costmap-cell-count-explosion) / [#23](#23-wsl2-gpu-render-cost-gazebo-gui--cameras) |
| `web_dashboard.py` prints "Running on ..." then vanishes | [#24](#24-dashboard-process-dies-silently) |
| fleet_manager retries once then silently stops forever, EVEN after #17's fix | [#25](#25-17s-fix-_one_shot-still-silently-died-under-real-wsl-load) |
| Robot starts navigating, then mid-route every goal starts getting rejected | [#26](#26-bt_navigators-lifecycle-bond-breaks-mid-navigation-under-load) |

---

## 1. Only 1 robot spawns — double-namespaced nodes

**Symptom:** with `num_robots:=3`, only robot1 actually came up; logs showed node names
like `robot2.robot2.robot_state_publisher`.

**Cause:** `robot_state_publisher_node` already had `namespace=ns` set directly. Wrapping
it in `PushRosNamespace(ns)` as well applied the namespace twice.

**Fix:** nodes that accept `namespace=` directly (`robot_state_publisher`, the sensor
bridge) are grouped in a plain `GroupAction` with **no** `PushRosNamespace`.
`PushRosNamespace` is reserved for `IncludeLaunchDescription`s whose own nodes (like
nav2_bringup's `navigation_launch.py`) don't set `namespace=` themselves — see #3.

---

## 2. `controller_manager` never comes up in fleet mode

**Symptom:** hangs forever on `Waiting for data on 'robot_description' topic`.

**Cause:** `gz_ros2_control`'s `GazeboSimROS2ControlPlugin` wasn't namespaced, so it
looked for a plain `/robot_description` topic that never existed (the real one was
`/robotN/robot_description`).

**Fix:** added a `namespace_controller_manager` xacro arg; when true, wraps the plugin's
`<ros>` block with `<namespace>${robot_name}</namespace>` and explicit `/tf`, `/tf_static`
remaps. **Gotcha:** must gate this with `<xacro:if value="$(arg ...)">` using the raw
`$(arg ...)` substitution — `${property == 'true'}`-style string comparison silently
always evaluated false.

---

## 3. `controller_server` crashes: "No critics defined for FollowPath"

**Symptom:** SIGABRT during Nav2 bringup, controller_server aborts.

**Looked like:** a YAML formatting issue with the multi-line `critics` list (fixed that
too, but it wasn't the real bug).

**Actual cause:** `nav2_navigation`'s `IncludeLaunchDescription` (nav2_bringup's
`navigation_launch.py`) was not wrapped in `PushRosNamespace(ns)`. Its own `Node()`
definitions don't set `namespace=` themselves — they rely entirely on an external
`PushRosNamespace`. Its params ARE rewritten with `root_key=namespace`
(`nav2_common.RewrittenYaml`), nesting the whole params file under a `robot1:` key that
only matches a node whose fully-qualified name is `/robot1/controller_server`. Without
the namespace wrapper, the node came up as plain `/controller_server`, which no longer
matched the rewritten params — so it silently fell back to defaults, including an empty
`critics` list.

**Fix:** `GroupAction(actions=[PushRosNamespace(ns), IncludeLaunchDescription(...)])`.

**Lesson:** when an `IncludeLaunchDescription` from someone else's package behaves oddly
under namespacing, go read that package's own launch file source
(`find /opt/ros -iname "*navigation_launch*"`) rather than guessing — this exact bug cost
the most time in the whole session because the crash message pointed at the params file,
not the missing namespace wrapper.

---

## 4. Controller YAML silently not applied once namespaced

**Symptom:** `The 'type' param was not defined for 'joint_state_broadcaster'` — right
after fixing #2.

**Cause:** `amr_ros_dg_controllers.yaml` used plain top-level keys
(`controller_manager:`, `joint_state_broadcaster:`, ...), which only match a node whose
fully-qualified name is exactly that (no namespace). Once `controller_manager` itself
became namespaced (`/robot1/controller_manager`), the plain keys stopped matching.

**Fix:** wrapped the entire file under the `/**:` wildcard, which matches a node under
ANY namespace. This one file now works for both the unnamespaced single-robot path
(`gazebo.launch.py`) and every namespaced fleet robot.

---

## 5. `collision_monitor` crashes on empty polygon list

**Symptom:** SIGABRT on startup: `parameter_value_from failed ... No parameter value set`.

**Cause:** an empty YAML list (`polygons: []`, `observation_sources: []`) has no
inferrable element type for a ROS 2 parameter — some Nav2 nodes abort on this instead of
treating it as "no items".

**Fix:** don't hand-roll a minimal config with empty lists. Pulled nav2_bringup's own
default `collision_monitor` section (found via
`find /opt/ros -iname "*nav2_params*"` / reading the installed `nav2_bringup` share dir)
which has real, non-empty polygon/observation-source entries.

---

## 6. `docking_server` aborts bringup

**Symptom:** `"Charging dock plugins not given!"`, whole robot's lifecycle_manager aborts.

**Cause:** `navigation_launch.py` unconditionally brings up `docking_server` as a managed
node — it must configure() without error even though nothing in this simulation ever
sends a dock action.

**Fix:** added a stub `dock_plugins: ["simple_charging_dock"]` /
`simple_charging_dock: {plugin: "opennav_docking::SimpleChargingDock"}` registration —
exists only so the lifecycle node has a valid config, never actually used.

---

## 7. `bt_navigator` goals permanently rejected

**Symptom:** every `NavigateToPose` goal rejected with "Action server is inactive",
forever — even minutes after bringup.

**Looked like:** a lifecycle activation race (there IS a normal, self-resolving burst of
this exact message during the first few seconds of bringup while `lifecycle_manager` is
still sequentially activating nodes — don't confuse that transient burst with this).

**Actual cause (this instance):** `default_nav_to_pose_bt_xml: ""` in `nav2_params.yaml`.
An empty string is NOT the same as "use the default" — it left the BT navigator with no
tree to run at all.

**Fix:** removed the `default_nav_to_pose_bt_xml` / `default_nav_through_poses_bt_xml`
keys entirely so `bt_navigator` falls back to its own compiled-in default.

---

## 8. `/tf`, `/tf_static` never actually namespaced

**Symptom:** `ros2 topic list` showed only a global `/tf`, never `/robot1/tf`. Worse:
`ps aux` showed the process running with exactly the intended
`-r /tf:=tf -r /tf_static:=tf_static` CLI args, yet
`ros2 node info /robot1/robot_state_publisher` still listed `/tf` (unnamespaced) as a
publisher.

**Cause:** `robot_state_publisher`, `slam_toolbox`, and `gz_ros2_control`'s controller
manager all publish `/tf`/`/tf_static` on **hardcoded absolute topics** (leading slash),
which bypasses normal ROS 2 namespace resolution. A *relative* remap target (`"tf"`,
relying on `__ns:=/robotN` to then namespace it) does not reliably apply before/after the
remap in this situation.

**Fix:** remap straight to the fully-qualified absolute target:
`remappings=[("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")]` on every node that
touches TF (`robot_state_publisher`, `slam_toolbox`, the `gz_ros2_control` plugin's
`<ros>` block).

---

## 9. `ros2 daemon` staleness in WSL2

**Symptom:** `ros2 topic list` / `ros2 node list` intermittently empty or stale —
especially confusing when one terminal shows a topic and another, opened later, doesn't.

**Fix (roughly in order of how much it takes):**
```bash
ros2 daemon stop && ros2 daemon start
ros2 <cmd> --no-daemon          # bypass the daemon entirely for one call
```
If that's not enough (seen once, alongside an actual
`RTPS_TRANSPORT_SHM Error: Failed init_port`):
```bash
wsl --shutdown            # from PowerShell, NOT inside WSL
# then reopen a WSL terminal
rm -rf /dev/shm/fastrtps_*
```
**Lesson:** before concluding a topic genuinely doesn't exist, restart the daemon and
retry in the SAME terminal window/tab the pipeline is actually running in — commands run
from a different terminal app or a freshly-opened tab have, more than once in this
session, looked like "the topic doesn't exist" when the real problem was daemon/session
staleness, not the pipeline.

---

## 10. Local costmap can't find `odom` frame

**Symptom:** `local_costmap`: "Timed out waiting for transform from base_link to odom...
frame does not exist".

**Cause:** `diff_drive_controller`'s `tf_frame_prefix_enable` (default `true`) auto-
prefixes `odom_frame_id`/`base_frame_id` with the controller's own namespace when active
— so in fleet mode it was actually publishing `robot1/odom` → `robot1/base_link`, not the
plain `odom`/`base_link` every other config file in this repo assumes.

**Fix:** `tf_frame_prefix_enable: false` in `amr_ros_dg_controllers.yaml`. Namespacing is
handled entirely via the `/robotN/tf` topic (see #8), not frame_id text — this repo's
deliberate design.

---

## 11. Global costmap can't find `map` frame

**Symptom:** `global_costmap`: "Timed out waiting for transform from base_link to map".

**Cause:** `/robotN/map` never existed because `slam_toolbox_node`, run as a plain
`Node()`, never self-configured/activated — `async_slam_toolbox_node` is a ROS 2
**lifecycle** node and does not auto-activate like a normal node.

**Fix:** converted to `LifecycleNode` + the same explicit
`EmitEvent(ChangeState(...CONFIGURE))` → `RegisterEventHandler(OnStateTransition(...))` →
`EmitEvent(ChangeState(...ACTIVATE))` pattern used by slam_toolbox's own bundled
`online_async_launch.py` (found via `find /opt/ros -iname "*online_async*"`).
**Gotcha:** `OnStateTransition` is imported from `launch_ros.event_handlers`, not
`launch.event_handlers` — easy one-letter-package mistake that fails at import time.

---

## 12. slam_toolbox never publishes `/map`

**Symptom:** slam_toolbox activates cleanly, logs "Registering sensor: [Custom Described
Lidar]", then nothing — no crash, no further logs, `/map` never appears.

**Cause:** a hand-rolled 5-key parameter dict (`use_sim_time`, `odom_frame`,
`base_frame`, `map_frame`, `scan_topic`) is missing other required defaults
(mode/resolution/update intervals/etc.) that slam_toolbox's own config file provides.

**Fix:** load the full official `mapper_params_online_async.yaml` (from the installed
`slam_toolbox` share dir) as the base parameters, and only override the per-robot
frame/topic names on top of it.

---

## 13. Lidar frame_id mismatch in slam_toolbox

**Symptom:** `slam_toolbox`: "Message Filter dropping message: frame
'robot1/base_link/lidar_sensor' ... discarding message because the queue is full",
forever — no map ever builds.

**Cause:** sdformat's URDF→SDF conversion lumps a fixed-jointed child link (`lidar_link`)
into its parent for frame-naming purposes, so the bridged `LaserScan`'s `frame_id` came
out as a composite scoped name instead of the URDF's actual `lidar_link`, and no TF
transform exists for that composite name.

**Fix:** `<gz_frame_id>lidar_link</gz_frame_id>` inside the sensor's `<sensor>` block —
gz-sensors' explicit override to force the published `frame_id` to match what
`robot_state_publisher` actually publishes a transform for.

---

## 14. Planner: goal outside costmap bounds

**Symptom:** `NavfnPlanner`: `"Goal Coordinates of(x, y) was outside bounds"` (or, before
a `rolling_window`/size was set at all, the same for the START coordinates).

**Cause (two instances, same category):**
- No `rolling_window`/`width`/`height` at all → `global_costmap` defaults to a tiny area
  that may not even include the robot's own start pose.
- Later, `width`/`height` set too small relative to the world: with `width: 25` (12.5m
  half-width, rolling window centered on the robot), a goal ~13.65m away
  (e.g. station x=6.75 to x=-6.9 in a 15x20m world) fell outside the window.

**Fix:** size the rolling window comfortably past the world's own diagonal
(`width/height: 60` → 30m half-width, safely past this world's ~25m diagonal). See #22
for the CPU-cost side effect of doing this at too fine a resolution.

---

## 15. `cmd_vel` published to a dead-end topic

**Symptom:** Nav2 reports the goal succeeded (or fails cleanly) but the wheels never
physically move.

**Cause:** `collision_monitor`'s `cmd_vel_out_topic` defaulted to `"cmd_vel"`, which
nothing subscribes to — the loaded `diff_drive_controller` (a ros2_control controller,
not a plain node) actually listens on `<controller_name>/cmd_vel`
(`diff_drive_controller/cmd_vel`), relative to its controller_manager's namespace.

**Diagnosis technique that found it:** `ros2 topic info /robot1/diff_drive_controller/cmd_vel -v`
— shows publisher/subscriber counts and types on a topic; zero subscribers to the topic
`collision_monitor` was actually using was the smoking gun.

**Fix:** `cmd_vel_out_topic: "diff_drive_controller/cmd_vel"`.

---

## 16. Twist vs. TwistStamped type mismatch

**Symptom:** even after #15, still zero effective connection — `ros2 topic info -v`
showed `collision_monitor` publishing `geometry_msgs/msg/Twist` while
`diff_drive_controller` subscribed `geometry_msgs/msg/TwistStamped` on the exact same
topic NAME. Matching names with mismatched types connect to nothing, silently, no error.

**This is a recurring bug class in this whole project** — first hit (in an earlier,
pre-fleet session) between `wasd_teleop.py` and `diff_drive_controller`; recurred here
between `collision_monitor` and `diff_drive_controller`.

**Fix:** `enable_stamped_cmd_vel: true` on **every** Nav2 node that touches `cmd_vel` in
the chain — `controller_server`, `velocity_smoother`, AND `collision_monitor`. Missing it
on just one of the three silently breaks the chain at that link.

**Lesson:** whenever a `cmd_vel`-shaped topic "isn't working" with zero errors, check
`ros2 topic info <topic> -v` for a message TYPE mismatch before anything else — ROS 2
does not warn about this.

---

## 17. `create_timer` is periodic, not one-shot

**Symptom:** `fleet_manager.py`'s retry logic ("no free station, retrying in 1s" / "server
not ready, retrying in 2s") fired exactly ONCE per robot and then silently never retried
again, even though the condition it was waiting on (e.g. bt_navigator becoming active)
resolved seconds later.

**Cause:** `rclpy`'s `create_timer()` creates a PERIODIC timer. Calling the retry function
from inside the timer callback without cancelling the timer first means the ORIGINAL
timer is still running independently — the intent was "retry once after N seconds", the
actual behavior was "the function you called overwrote nothing, and the old timer just
kept firing into a state that no longer needed it, or got garbage collected because
nothing held a reference".

**Fix:** a small `_one_shot(delay_sec, fn)` helper:
```python
def _one_shot(self, delay_sec, fn):
    timer_ref = []
    def _fire():
        timer_ref[0].cancel()
        fn()
    timer_ref.append(self.create_timer(delay_sec, _fire))
```

---

## 18. Zero-delay retry loop on goal rejection

**Symptom:** log showed hundreds of `assigned → rejected` pairs within ~90 milliseconds,
robot never actually starts navigating.

**Cause:** `_on_goal_response`, on `goal_handle.accepted == False`, immediately called
`_assign_next_task(task)` again with **no delay at all** — unlike the "no free station"
path, which already used `_one_shot` (see #17). If the rejection reason is transient
(e.g. `bt_navigator` not fully active yet), this forms a genuine zero-delay hot loop.

**Fix:** wrap the retry in the same `_one_shot(2.0, ...)` pattern used elsewhere in this
file.

**Lesson:** any retry-on-failure path needs the SAME "wait before retrying" treatment as
every other retry path in the same file — it's easy to fix this bug in one place and
leave an identical one two functions away.

---

## 19. Lidar collision geometry embedded in the chassis

**Symptom:** robot completely frozen — `/odom` position never changes at all, no errors
anywhere in the logs.

**Cause:** `lidar_link` had its own `<collision>` cylinder. Because `lidar_joint` is a
`type="fixed"` joint, dartsim/sdformat LUMPS that collision into `base_link`'s rigid body.
When the lidar's mount position was tuned close to or past the chassis surface (chasing a
"looks flush" visual placement), that lumped collision geometry started interpenetrating
the chassis mesh / ground, and the physics solver effectively locked the robot in place.

**Fix (first pass):** removed `<collision>` from `lidar_link` entirely — a small sensor
puck doesn't need physics collision. **This alone did not fully fix it** — see #20.

---

## 20. Lidar self-detection with a 360deg sweep

**Symptom:** after #19's collision removal, robot STILL frozen. Diagnosis:
`cmd_vel_smoothed` had real, nonzero velocity commands flowing (planner → smoother both
fine), but `diff_drive_controller/cmd_vel` (collision_monitor's output) was completely
empty — `collision_monitor` was withholding all velocity. Its own log showed "Robot to
approach for 1.2 seconds away from collision".

**Cause:** gz-sim's `gpu_lidar` ray-casts against the **rendered/visual** mesh, not
collision geometry (so #19's collision removal was irrelevant to this). With the lidar
mounted flush against/inside the chassis surface and a full 360-degree sweep
(`min_angle: -pi`, `max_angle: pi`), rays pointed backward/sideways from that position hit
the robot's own visible chassis at near-zero range. `collision_monitor` correctly read
that as an imminent collision and refused to move.

**Fix options used, in order:**
1. Raise the lidar well clear of the chassis dome (worked, but visually "floating").
2. **Narrow the sweep to 120 degrees forward-only** (`min_angle`/`max_angle` = ±60deg,
   `samples: 120`) — since the robot only ever drives forward with Nav2, nothing of the
   chassis sits inside that forward cone even when mounted flush/embedded, so the
   original "flush against the surface" visual placement became safe again.

**Lesson:** a visually-flush sensor mount and a 360-degree sweep are in tension for any
sensor mounted anywhere except the exact highest point of a convex chassis — narrowing
the FOV to match the sensor's actual mounting geometry is often the real fix, not endless
z/x nudging.

---

## 21. Controller spawner failures under load at scale

**Symptom:** at `num_robots:=10`, several robots' controller spawners logged
`Failed to acquire lock after multiple attempts` then
`process has died [pid ..., exit code 1]` — those robots' wheels never moved.

**Cause:** WSL2's CPU/IO gets saturated bringing up N simultaneous full stacks
(controller_manager + Nav2 + SLAM Toolbox, each) at once. The spawner's service calls to
its OWN robot's controller_manager (a per-robot resource, not shared across robots)
started timing out purely from system load, not a real logic conflict.

**Fix:** raised `--controller-manager-timeout` on both spawners from the 10s default to
60s — wait longer instead of dying. Consider also reducing `num_robots` while iterating,
and see #23 for reducing the load itself.

---

## 22b. `gz sim` orphaned by experiment_manager's own shutdown (compounds #21/#23)

**Symptom:** controller spawner failures like #21 above kept recurring even at
`num_robots:=3` (well below the ~10-robot threshold #21 was written for), and got worse
over a session, not better. `ps aux` on an otherwise-idle machine showed multiple `gz
sim` processes still running from PAST experiment runs, each still burning 60-150% CPU,
started minutes to hours earlier.

**Cause:** `experiment_manager/run_experiment.py`'s shutdown path sent `SIGINT` to its
`ros2 launch amr_ros_dg fleet.launch.py` subprocess, and on a 15s timeout escalated to
`proc.kill()` (`SIGKILL`). `kill()` on `subprocess.Popen` only ever signals that ONE
process - `ros2 launch` itself is not `gz sim`, it's a Python process that spawns `gz
sim` (and every Nav2/SLAM node) as further children via its own launch actions. A
`SIGKILL` to the launch process does not cascade to those children; they're simply
reparented to init and keep running. So every experiment run that hit the 15s timeout
(common under WSL2 load - the very thing #21 already documents) left its `gz sim` alive
forever, and each surviving instance made the NEXT run's WSL2 load worse, compounding
into more spawner timeouts, in a spiral. This masqueraded as "robots don't move" with no
obvious root cause per-run, because the actual cause was contamination from EARLIER runs.

**Fix:** every subprocess `run_experiment.py` launches now starts its own process group
(`start_new_session=True`), and shutdown signals the whole group via `os.killpg` (SIGINT
first, SIGKILL on timeout) instead of the single launch process - see
`_popen_own_group`/`_terminate_group` in `run_experiment.py`. Confirmed live in WSL: a
90s single-robot run with this fix left zero `gz sim`/`ros2 launch`/`controller_manager`
processes behind (`ps aux` clean 3s after "done."), and the robot's odometry showed real
forward motion (~1.3 m/s) for the first time across every WSL run in this session.

**Lesson:** if a "wait, then kill" subprocess-shutdown pattern only signals the direct
child, always check what THAT child itself spawns before trusting `kill()` to clean up a
process tree - especially for anything invoked via `ros2 launch`, which is a process tree
by construction, not a single process.

---

## 22c. #22b's process-group fix wasn't sufficient by itself

**Symptom:** even after the `_popen_own_group`/`os.killpg` fix in #22b, a 3-robot run
still left 1-2 `gz sim` processes running after `run_experiment.py` printed "done."

**Cause:** `ros2 launch` puts each of ITS OWN launched actions - `gz sim` included - into
a SEPARATE process group internally, not into the process group of the outer `ros2
launch` process itself. This is deliberate on `ros2 launch`'s part: it runs its own
multi-stage graceful shutdown per action (SIGINT, wait ~5s, escalate to SIGTERM, wait
more, escalate to SIGKILL - visible directly in its log output as e.g. "process failed to
terminate '5' seconds after receiving 'SIGINT', escalating to 'SIGTERM'"), which requires
being able to signal each action independently. So #22b's `os.killpg(pgid_of_fleet_proc,
...)` only ever reliably reaches the `ros2 launch` process itself, not `gz sim` - it just
usually worked because `ros2 launch`'s own cascade finished within our 15s top-level
timeout. Under heavier load (3 robots' full Nav2+SLAM+Gazebo teardown), its cascade can
still be mid-flight when our timeout fires and SIGKILLs the launch process's group -
`gz sim`, already outside that group, survives as an orphan.

**Fix:** don't try to out-guess `ros2 launch`'s internal process-group behavior -
`run_experiment.py`'s `_reap_stray_gz_sim()` unconditionally runs `pkill -9 -f 'gz sim
.*warehouse_fleet_world.sdf'` after every shutdown sequence finishes, regardless of how
gracefully it went. Safe specifically because this repo's whole experiment-runner design
assumes one experiment runs at a time (no parallel runs to accidentally kill).

**Lesson:** process-group-based signal propagation only works if you control (or have
verified) how every layer in the tree groups its own children - `ros2 launch` doesn't
inherit the group you expect, and probably neither do other process-supervisor-style
tools. When you can't verify that, an unconditional, narrowly-scoped `pkill -f` cleanup
pass after the fact is more robust than a "smarter" signal-based approach that depends on
an assumption you can't check from outside.

---

## 22. Global costmap cell-count explosion

**Symptom:** after fixing #14 by raising `width`/`height` from 25 to 60 (at the original
`resolution: 0.05`), robots barely moved at all — "2 minutes, moved a little."

**Cause:** cell count scales with `(width/resolution)^2`. `25m / 0.05` = 500x500 = 250k
cells; `60m / 0.05` = 1200x1200 = **1.44M cells, ~5.8x more, PER ROBOT**, updating every
costmap cycle. Fixing the "outside bounds" bug this way silently created a severe
performance regression.

**Fix:** coarsen `resolution` to `0.1` alongside the larger `width`/`height` — `60m/0.1` =
600x600 = 360k cells, back in the same ballpark as the original per-robot cost while
keeping full world coverage. Also lowered `controller_frequency` (20→10Hz) and
`local_costmap`'s `update_frequency`/`publish_frequency` (5/2 → 3/1Hz) to cut compute
further — all still comfortably responsive for a 2 m/s robot.

**Lesson:** whenever you widen a rolling costmap window to fix an "outside bounds" error,
check whether resolution needs to coarsen proportionally — the fix for one bug is a
textbook way to introduce a performance bug.

---

## 23. WSL2 GPU render cost: Gazebo GUI + cameras

**Symptom:** even after #21/#22, `ros2 topic hz /clock` showed a very low, bursty rate
(e.g. ~11Hz with huge std-dev/gaps) — meaning the simulation itself was running far
slower than real-time, not that any single node was misconfigured.

**Diagnosis:** `top` showed two processes using 130-230%+ CPU EACH, both labeled `ruby` in
the truncated command column — actually `gz sim server` and `gz sim gui` (Gazebo's own
physics + GUI rendering), while every ROS/Nav2 process individually used only 9-20%.

**Contributing factors, roughly in order of impact:**
1. The Gazebo **GUI window** itself (rendering the 3D scene) — real, measurable CPU cost
   on top of the physics server, largely from WSL2/WSLg's GPU-forwarding overhead.
2. Per-robot **GPU-rendered cameras** (`d435i` @ 30Hz, `floor` @ 15Hz) — real image
   rendering, not just a topic.
3. `wsl --update` (WSL 2.6.1.0 → 2.7.11.0, WSLg 1.0.66 → 1.0.73.2) measurably helped
   (`/clock` rate roughly doubled at num_robots:=1, from ~11Hz baseline behavior to
   57-72Hz) — worth doing once, cheaply, before further tuning.
4. `.wslconfig` memory/processor caps: bumping from WSL2's default allocation to
   `memory=24GB, processors=12` (matched to the host's actual specs) reduced load average
   from ~21 (badly oversubscribed on 12 cores) to ~4.5 at idle.

**Fixes implemented (all opt-in, don't change simulation logic):**
- `headless:=true` launch arg (default) — skips `gz sim gui` entirely (`-s` flag). The web
  dashboard's camera/lidar views work identically either way, so the GUI window is often
  redundant for fleet testing.
- `enable_cameras:=false` launch arg — skips the `d435i`/`floor` `<sensor>` elements (the
  links/meshes stay, only the expensive GPU render is skipped). Nav2/SLAM only need the
  lidar; the ultrasonic ring and cameras are extra, not load-bearing.
- `.wslconfig` at `C:\Users\<you>\.wslconfig` with explicit `memory`/`processors`/`swap`.

**If you need the GUI window anyway** (e.g. to visually place a sensor or watch behavior
directly), expect it to cost real performance — test with fewer robots first
(`num_robots:=1` before `:=3`), and check `ros2 topic hz /clock` as the ground-truth
signal for "is the sim actually keeping up," not wall-clock guesses about robot speed.

---

## 24. Dashboard process dies silently

**Symptom:** `web_dashboard.py` printed its full Flask startup banner
(`Running on http://0.0.0.0:8080` etc.) but was unreachable moments later —
`ps aux | grep web_dashboard` showed no such process at all, with no error printed
anywhere.

**Cause (most likely, not fully confirmed):** heavy concurrent load from a multi-robot
fleet run (see #21-23) starving/killing a freshly-started, low-priority process. `dmesg`
did not show an OOM-killer entry, but `dmesg` access is often restricted/incomplete
inside WSL2, so this isn't conclusive either way.

**Workaround:** just restart it (`ROBOT_LIST=... ros2 run amr_ros_dg web_dashboard.py`)
after the fleet has finished its heaviest startup burst, and keep an eye on whether it
survives past ~30s.

---

## 25. `#17`'s "fix" (`_one_shot`) still silently died under real WSL load

**Symptom:** identical to #17's original symptom, on a clean host (no leftover `gz sim`,
per #22b/#22c) - `navigate_to_pose server not ready yet, retrying in 2s` logged exactly
once per robot, then fleet_manager never sent another goal, ever, for the rest of a
150s+ run. No exception, no traceback anywhere in the log.

**Cause:** `_one_shot`'s fix for #17 (self-cancelling `create_timer()`) was real and
necessary, but not sufficient. Every retry called `self.create_timer(...)` fresh, from
inside a callback the `MultiThreadedExecutor` was already running - adding a NEW timer
entity to a running executor from within one of its own callbacks. rclpy is supposed to
wake its `rcl_wait()` for this via a guard condition, but under the same heavy CPU
contention documented in #21/#22b/#22c, that wake-up was apparently not reliably timely:
confirmed live, twice, in separate debugging sessions, with the executor's wait-set
seemingly never getting nudged to notice the new timer at all for the rest of the run.

**Fix:** stop calling `create_timer()` after node startup, at all. Every robot now gets
ONE persistent poll timer (0.2s period) created once in `__init__`, already sitting in
the executor's wait-set from the very first spin. Deferring work (a retry, the 2s
pickup/dropoff pause) is now just setting a plain Python attribute
(`task.pending_fn`/`task.pending_fire_at`) that the existing timer picks up on its next
tick - never a new entity, so there's nothing for the executor to need waking up for.
See `fleet_manager.py`'s `_schedule`/`_poll_pending`.

**Validated live in WSL2:** after this fix, the identical retry-then-silence symptom was
gone - `navigate_to_pose server not ready yet, retrying in 2s` now recurs every ~2s for
as long as needed (confirmed spread over 90+ seconds in one run), and a single robot
successfully completed a full pickup-station route end to end (`arrived at
'output_station_10', drop off (2s)... done at 'output_station_10', releasing`) in a
400s run - see #26 for why 400s, not 90s, was needed to see that.

---

## 26. `bt_navigator`'s lifecycle bond breaks mid-navigation under load

**Symptom:** after #25's fix, a robot would successfully get its Nav2 stack active, get
a goal accepted, and start genuinely navigating (`bt_navigator: Begin navigating from
current location...`) - then, tens of seconds later, mid-route: `[ActionServer] Aborting
handle... Destroying bond (bt_navigator) to lifecycle manager`, after which EVERY
subsequent goal was rejected with `Action server is inactive` for the rest of the run.
`bt_navigator` itself never crashed or logged an error before this.

**Cause:** `nav2_lifecycle_manager` defaults to `bond_timeout: 4.0` - each managed node
(bt_navigator included) must publish a ROS bond heartbeat at least that often or
`lifecycle_manager` assumes it died and tears it down. Under the CPU contention
documented in #21/#22b/#22c/#25, a CPU-starved `bt_navigator` can miss that 4s window
while genuinely alive and mid-route, not dead - `lifecycle_manager` has no way to tell
the difference and deactivates it anyway. Once deactivated, nothing in this codebase
ever reactivates it, so the robot is permanently stuck for the rest of the run.

**Fix:** `lifecycle_manager_navigation: {ros__parameters: {bond_timeout: 0.0}}` added to
`config/nav2_params.yaml`, disabling bond monitoring entirely. This is the standard
recommendation for constrained/simulated hosts where a missed soft-real-time heartbeat
doesn't actually mean the node died - `lifecycle_manager` still fully supervises
bringup/shutdown either way, it just stops assuming silence under load means death.

**Also worth knowing (not itself a bug, just slower than expected):** even with #25 and
this fix, a single robot took ~130-150s of WALL-CLOCK time to drive an ~11m route
commanded at ~1.2 m/s - i.e. the effective sim-time/wall-time ratio under this WSL2
headless setup is roughly 0.07-0.1, matching Phase 1's documented "simulated-vs-wall-
clock rate is low" finding. Short test durations (under ~200s for even one robot's first
full route, more for a multi-robot fleet) will look like a failure that isn't one -
budget accordingly, and prefer checking `robot_<ns>.csv` for STEADY non-zero velocity
over a "did it complete" pass/fail read when a run's duration might just be too short.

---

## 27. `rclpy.shutdown()` raises `RCLError: rcl_shutdown already called` on SIGINT teardown

**Symptom:** a node exits with code 1 and a traceback ending in
`rclpy._rclpy_pybind11.RCLError: failed to shutdown: rcl_shutdown already called on the
given context`, even though the node otherwise ran and shut down correctly (data already
flushed/closed before this point).

**Cause:** the common `main()` pattern - `rclpy.spin(node)` in a `try`, unconditional
`rclpy.shutdown()` in the `finally` - assumes `rclpy.shutdown()` is only ever called once.
Under `ros2 launch`'s SIGINT-driven teardown, rclpy's own default signal handler can shut
the context down first (as part of unblocking `spin()`), so the explicit call in `finally`
raises because the context is already dead. Harmless in effect (happens after
`destroy_node()`, after any data has already been written), but noisy and technically an
unhandled exception exit code.

**Fix (applied in `fleet_coordination/decentralized_agent.py`, Phase 5):** guard the call
with `if rclpy.ok(): rclpy.shutdown()`. **Not yet applied elsewhere** - the identical
unguarded pattern still exists in `fleet_manager.py`, `event_logger_node.py`,
`health_monitor.py`, `fault_injector.py`, `web_dashboard.py`, and `wasd_teleop.py` (all
pre-date Phase 5); worth a small cleanup pass if the noisy exit code ever needs to be
clean, but out of scope for any single phase so far.

---

## 28. `experiments/` silently written under `install/.../lib/...` instead of the repo root

**Symptom:** a run completed successfully (`[run_experiment] done. ... dir=...`) but the
printed `dir=` was something like
`install/experiment_manager/lib/experiments/2026-08-11_<mode>_001` - inside the colcon
`install/` tree - instead of the repo's own `experiments/` directory. `ls experiments/`
at the repo root didn't show the new run at all, even though it clearly ran and Gazebo
genuinely moved robots.

**Cause:** `experiment_manager/run_paths.py` computed `REPO_ROOT =
Path(__file__).resolve().parents[3]` - a FIXED parent-directory count from wherever the
running `run_paths.py` file physically is. This only lands on the actual repo root when
colcon used `--symlink-install`: the installed file is then a symlink, and
`.resolve()` follows it back to the real file under `src/experiment_manager/
experiment_manager/run_paths.py`, where 3 parents up genuinely is the repo root. A plain
(non-symlink) `colcon build --packages-select experiment_manager` copies the file
instead, landing it at
`install/experiment_manager/lib/python3.12/site-packages/experiment_manager/
run_paths.py` - a DIFFERENT depth - so the same fixed `parents[3]` silently pointed at
`install/experiment_manager/lib` instead. This had been invisibly correct for the whole
project up to this point only because `experiment_manager` itself had never been
rebuilt without `--symlink-install` before; the first plain rebuild of that specific
package (Phase 7, adding `agent_backend` wiring) broke it.

**Fix:** replaced the fixed-depth computation with `_find_repo_root()`, which walks up
from `__file__.resolve()` looking for a `.git` directory - correct regardless of
symlink-install vs. copy-install, and regardless of how many directories deep colcon's
Python package layout happens to nest a given package's installed files. Covered by two
new unit tests (`test_find_repo_root_locates_git_dir_regardless_of_nesting_depth`,
`test_find_repo_root_raises_when_no_git_dir_found`) that construct a fake deeply-nested
`install/.../site-packages/...` tree so this can't silently regress again the same way.

**Lesson:** never derive "the repo root" (or any similarly load-bearing path) from a
fixed number of `Path.parents[N]` hops off an installed package's `__file__` - the
correct hop count is an accident of the current build/install method, not a stable
contract. Search for an actual marker (`.git`, a known sibling file) instead.

---

## 29. A negotiation deadline silently never fires because it shares `decentralized_agent.py`'s single deferred-work slot

**Symptom:** a robot correctly broadcasts a Phase 8 `OFFER` (task-transfer negotiation)
after its battery drops low, a peer correctly `BID`s on it, but the negotiation never
resolves - no `ACCEPT`/`REJECT`/`COMMIT`/`TIMEOUT` is ever published, and the initiating
robot never negotiates again for the rest of the run (not just for that one task - every
subsequent low-battery task too).

**Cause:** `decentralized_agent.py`'s single `_pending`/`_schedule()` slot
(TROUBLESHOOTING.md #25's one-persistent-poll-timer pattern) was built on the assumption
that a robot only ever needs ONE deferred action pending at a time - true through Phase
5-7, false as soon as Phase 8 added a second independent kind of deferred work: a
negotiation's 5-second deadline running concurrently with whatever the robot's normal
task cycle also has scheduled (a nav-goal-not-ready retry, the pickup/dropoff pause,
etc.). Confirmed live: `_initiate_transfer()` scheduled the negotiation's
`_resolve_transfer` callback into the one slot, and microseconds later a Nav2 action
result arrived and its callback (`_on_nav_result`) scheduled the pickup/dropoff pause
into that SAME slot, silently overwriting the negotiation callback that was never called
- `_active_conversation` stayed set forever, which is also what permanently blocked
`_maybe_offer_transfer()`'s "no negotiation already in flight" guard from ever allowing
another offer.

**Fix:** gave negotiation its own independent slot (`_negotiation_pending`,
`_schedule_negotiation()`), polled alongside the original one in the same
`_poll_pending()` tick rather than sharing it. The two are never used for the same
purpose, so no coordination between them is needed - just two independent
fire-if-due checks per tick.

**Validated live, same seed, before and after the fix (2026-08-11):** before: a
negotiation's `OFFER`→`BID` was captured in `negotiations.csv` with no resolution row at
all - a permanently dangling conversation. After: rerunning the identical scenario
(2 robots, seed 5, LOW_BATTERY fault on robot1 at the same offset) produced a clean
`OFFER`→`TIMEOUT` pair (this run's bid didn't happen to land in time, which is itself
fine - the point being verified was that `_resolve_transfer` actually fires and
publishes a terminal message either way, not that a bid must win).

---

## General debugging patterns that worked (worth reusing)

1. **Read the actual installed source of any package whose launch/config behavior is
   surprising**, rather than guessing: `find /opt/ros -iname "*<thing>*"` and `cat` it.
   This resolved #3, #11, #5/#6 (nav2_bringup's own default `nav2_params.yaml`), and #12.
2. **`ros2 topic info <topic> -v`** for type + pub/sub counts is the single best tool for
   "topic exists but nothing is happening" bugs (#15, #16).
3. **Change one thing, rebuild, relaunch, grep/echo one specific thing, read the raw
   output before guessing the next fix.** Every bug above was found this way, not by
   reasoning from the code alone — several (esp. #3, #20) had a misleading first
   hypothesis that a live log/topic check disproved.
4. **`ros2 topic hz /clock`** is the ground-truth check for "is the simulation actually
   keeping up with real time" — far more reliable than eyeballing robot speed, especially
   before concluding a *logic* bug when the real issue is *performance* (#22, #23).
5. When editing files under WSL that are supposed to be edited from Windows (or vice
   versa), confirm the path in `~/ros2_ws/src/<pkg>` is a **symlink** to the Windows path,
   not an independent copy — a stale copy silently ignoring every edit wasted real time
   in this session before it was caught (`ls -la ~/ros2_ws/src/amr_ros_dg`, expect
   `-> /mnt/d/...` in the output).

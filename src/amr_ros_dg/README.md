# amr_ros_dg

ROS 2 robot-description package for **AMR-ROS-DG**, a two-wheel differential-drive
autonomous mobile robot (AMR) with four free-swiveling caster wheels, generated from
a SolidWorks assembly via the SW2URDF exporter and migrated from the original ROS 1
(catkin) export to ROS 2 (`ament_cmake`).

![AMR product render](../../docs/renders/amr-product-render-front-angle.jpg)

## What this package provides

- A cleaned, parametric **Xacro** description (`urdf/amr_ros_dg.urdf.xacro`) with every
  mass, center-of-mass, inertia tensor, and joint origin taken verbatim from the
  SolidWorks mass-properties export — nothing is invented or guessed.
- Correctly scaled, per-part **meshes** (`meshes/*.STL`), in meters, each centered on
  its own link's coordinate system.
- A ROS 2 **launch file** (`launch/display.launch.py`) that starts
  `robot_state_publisher`, `joint_state_publisher_gui`, and `rviz2`.
- A saved **RViz2 config** (`rviz/amr_ros_dg.rviz`).
- A **Gazebo Sim (Harmonic) launch file** (`launch/gazebo.launch.py`) with `ros2_control`
  wired to the two drive wheels via `gz_ros2_control` + `diff_drive_controller`.
- A **WASD keyboard teleop node** (`scripts/wasd_teleop.py`) to drive the robot.
- A **web control dashboard** (`scripts/web_dashboard.py`) — live camera feeds, an
  ultrasonic radar view, wheel telemetry, and drive control with adjustable speed limits,
  all in one browser page (see "Web dashboard" below).
- The full **ROS 1 original export and every intermediate broken mesh attempt**,
  kept under `legacy/` for traceability (see `legacy/README.md`).

## Sensors (added on top of the CAD, not exported from SolidWorks)

None of these exist in the original assembly, so there's no CAD coordinate system to
pull exact numbers from - placements are estimated from the chassis envelope and marked
in the xacro with the exact assumption made. **Confirm/correct in RViz before trusting
them for anything beyond a first look** (`ros2 launch amr_ros_dg display.launch.py`).

| Link | Type | Mount | Placement basis |
|---|---|---|---|
| `d435i_link` | Intel RealSense D435i (90x25x25mm, real spec) | Front, fixed, forward-facing | Flush with front face (x=+0.397m), height estimated (z=-0.05m) |
| `floor_camera_link` | Downward barcode/floor camera (generic placeholder box) | Underside, fixed, straight down | 8cm above floor, as given - floor computed from wheel joint z minus wheel radius |
| `ultrasonic_1..12_link` | Ultrasonic ring (generic placeholder cylinders) | Perimeter, fixed, 30 deg apart, radial | Ellipse fit at 92% of chassis half-extents, height estimated (z=-0.15m) |

Everything estimated is called out as such directly in `urdf/amr_ros_dg.urdf.xacro`
(search for "ESTIMATE"). The one assumption that most needs visual confirmation is
`front_sign` - whether +X is actually the robot's front; flip it to `-1` in the xacro
if the D435i turns out to be facing the back in RViz.

All three now have real gz-sim sensor plugins wired up (see "Viewing sensor data in
Gazebo" below) - the two cameras publish `sensor_msgs/msg/Image`, and each ultrasonic is
a single-sample `gpu_lidar` (gz sim has no dedicated ultrasonic sensor type) publishing
`sensor_msgs/msg/LaserScan` with one range reading. Resolutions/FOVs/ranges are simulation
defaults, not real hardware specs (no ultrasonic/floor-camera hardware has been chosen
yet) - only the D435i's FOV is a public spec value.

## Robot structure

Two direct-drive wheels are mounted straight to the chassis (motor+gearbox housing is
fixed, only the output shaft/wheel rotates). Four passive casters (front-left,
front-right, back-left, back-right) each have two joints: a swivel joint (continuous,
about Z) and a rolling joint (continuous, about the wheel's local rolling axis).

```
base_link
  |-- left_wheel_joint   (continuous, axis 0 1 0)   -> left_wheel
  |-- right_wheel_joint  (continuous, axis 0 1 0)   -> right_wheel
  |-- caster_f_l_joint   (continuous, axis 0 0 1)   -> caster_f_l
  |     `-- caster_wheel_f_l_joint (continuous)     -> caster_wheel_f_l
  |-- caster_f_r_joint -> caster_f_r -> caster_wheel_f_r_joint -> caster_wheel_f_r
  |-- caster_b_r_joint -> caster_b_r -> caster_wheel_b_r_joint -> caster_wheel_b_r
  `-- caster_b_l_joint -> caster_b_l -> caster_wheel_b_l_joint -> caster_wheel_b_l
```

11 links, 10 joints, single root (`base_link`) — verified against the exporter's own
CSV/URDF output.

## Package layout

```
amr_ros_dg/
├── package.xml, CMakeLists.txt   ROS 2 (ament_cmake) package definition
├── run_in_wsl.sh                 one-shot build+launch script for WSL/Ubuntu (RViz)
├── run_gazebo_in_wsl.sh          one-shot build+launch script for Gazebo Sim + ros2_control
├── urdf/
│   └── amr_ros_dg.urdf.xacro     the robot description (source of truth), now includes
│                                  <ros2_control> + <gazebo><plugin> blocks for Gazebo
├── meshes/                       11 STL files, one per link, in meters
├── launch/
│   ├── display.launch.py         robot_state_publisher + joint_state_publisher_gui + rviz2
│   └── gazebo.launch.py          Gazebo Sim + robot spawn + controller_manager spawners
├── rviz/
│   └── amr_ros_dg.rviz           saved RViz2 view (Fixed Frame = base_link)
├── worlds/
│   └── amr_test_world.sdf        Gazebo world used by gazebo.launch.py (ground plane, sun,
│                                  Sensors system plugin for the camera/lidar sensors)
├── config/
│   ├── joint_names_amr_ros_dg.yaml       joint name list (legacy, from the SW export)
│   ├── amr_ros_dg_controllers.yaml       ros2_control: joint_state_broadcaster + diff_drive_controller
│   └── nav2_params.yaml                  full Nav2 stack config, used by fleet.launch.py
├── scripts/
│   ├── wasd_teleop.py             WASD + space keyboard teleop (publishes TwistStamped)
│   ├── web_dashboard.py           Flask control dashboard (cameras, lidar, ultrasonic, drive)
│   └── fleet_manager.py           multi-robot task assignment + station reservation
├── web/
│   └── dashboard.html            the dashboard's page (self-contained, no external deps)
├── textures/                     empty, reserved by the SolidWorks exporter
├── legacy/                       full history of every export attempt (see below)
├── QUICKSTART.md                 fastest path to a running multi-robot fleet
└── TROUBLESHOOTING.md            every bug hit building the fleet sim, symptom -> fix
```

## Build & run (WSL / Ubuntu 24.04, ROS 2 Jazzy)

The fastest path is the bundled script — it installs ROS 2 if missing, copies the
package into a colcon workspace, builds it, and launches RViz2:

```bash
bash "/mnt/c/Users/<you>/OneDrive/Desktop/AMR-ROS/ros2-package/amr_ros_dg/run_in_wsl.sh"
```

Or manually:

```bash
mkdir -p ~/ros2_ws/src
cp -r "/mnt/c/Users/<you>/OneDrive/Desktop/AMR-ROS/ros2-package/amr_ros_dg" ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select amr_ros_dg --symlink-install
source install/setup.bash
ros2 launch amr_ros_dg display.launch.py
```

This opens RViz2 with the robot model loaded and a `joint_state_publisher_gui` window
for manually driving each wheel/caster joint to sanity-check the kinematics.

## Driving the robot in Gazebo (Gazebo Sim / ros2_control)

Terminal 1 — build and launch the simulation:

```bash
bash "/mnt/c/Users/<you>/OneDrive/Desktop/AMR-ROS/ros2-package/amr_ros_dg/run_gazebo_in_wsl.sh"
```

This spawns the robot in Gazebo Sim ("Harmonic", the Gazebo that ships with ROS 2
Jazzy — not classic Gazebo) and starts `joint_state_broadcaster` +
`diff_drive_controller` via `gz_ros2_control`.

Terminal 2 — drive it:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run amr_ros_dg wasd_teleop.py
```

```
  w : forward       s : backward
  a : turn left      d : turn right
  space : stop        q : quit
```

Each keypress sets a persistent velocity (like `teleop_twist_keyboard`) that keeps
being published until you press another key — press `space` any time to stop
immediately.

### Viewing sensor data in Gazebo

The world loaded by `gazebo.launch.py` is `worlds/amr_test_world.sdf`, not ros_gz_sim's
bundled `empty.sdf` — camera/lidar rendering needs the world to load gz-sim's `Sensors`
system plugin, which `empty.sdf` doesn't include. `launch/gazebo.launch.py` also brings
up a `ros_gz_bridge parameter_bridge` for every sensor topic automatically, so as soon as
Gazebo is up these are live ROS 2 topics:

```bash
ros2 topic list | grep -E 'camera|ultrasonic'
```

- **D435i camera** — `ros2 run rqt_image_view rqt_image_view /camera/d435i` (or add an
  Image display on that topic in RViz2)
- **Floor camera** — same, topic `/camera/floor`
- **Ultrasonic ring** — 12 topics `/ultrasonic_1/scan` .. `/ultrasonic_12/scan`
  (`sensor_msgs/msg/LaserScan` with a single range reading each, since gz sim has no
  dedicated ultrasonic sensor type):
  ```bash
  ros2 topic echo /ultrasonic_1/scan --field ranges
  ```

### Web dashboard

A single-page control dashboard combining both camera feeds, a live ultrasonic radar
view, wheel telemetry, and drive control (keyboard or on-screen D-pad, with speed-limit
sliders) — the visual/remote equivalent of `wasd_teleop.py`. Run it in a third terminal
once Gazebo is up:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run amr_ros_dg web_dashboard.py
```

Then open **http://localhost:8080** in a browser (WSL2 forwards the port to Windows
automatically, so this works from a Windows browser too). Drive with `W`/`A`/`S`/`D` or
the on-screen buttons, `Space` to stop, and the two sliders cap linear/angular speed as a
percentage of the limits already in `config/amr_ros_dg_controllers.yaml`
(`linear.x.max_velocity` / `angular.z.max_velocity`) — not new numbers.

Pure HTTP polling under the hood (no websockets/socketio dependency): the browser polls
`/api/state` for telemetry and re-requests `/api/camera/<name>` on an `<img>` tag for a
motion-JPEG-style feed, and POSTs `/api/cmd` at 10Hz while a direction is held — a
watchdog on the node zeroes the published velocity if that POST stream stops for more
than 0.5s (browser tab closed, network drop), so a lost connection can't leave the robot
creeping.

### What's actuated vs. passive

Only `left_wheel_joint` and `right_wheel_joint` are in the `<ros2_control>` block
(velocity command interface — no PID gains needed at this layer, since a velocity
interface is driven directly by `gz_ros2_control`). All four casters are left out of
`<ros2_control>` entirely; they roll and swivel passively under contact physics, using
the damping/friction values already in the URDF from CAD.

### Gazebo physics: caster wheels are tip-over support, not a traction point

The drive wheels alone provide traction (`mu=1.0`). The four casters are a **tip-over/
pitch backstop, not a permanent contact point** — their collision geometry sits ~0.5mm
above the drive-wheel contact plane on purpose, so a caster only touches the ground once
the chassis pitches or rolls enough to close that gap, the same way a shopping cart's
casters barely load until you lean it. This was necessary because a rigid physics
solver with no suspension modeled is sensitive to sub-millimeter contact-height
differences between wheels; letting all 6 wheels share one contact plane meant the very
light casters could end up bearing the chassis's weight instead of the drive wheels
(the "wheels spin, chassis doesn't move" symptom during initial Gazebo testing). See the
`GAZEBO PHYSICS NOTES` block at the top of `urdf/amr_ros_dg.urdf.xacro` for the full
history and numbers.

### Numbers used, and where they came from

- `wheel_separation: 0.579 m` and `wheel_radius: 0.075 m` in
  `config/amr_ros_dg_controllers.yaml` are measured directly from the CAD data
  (wheel joint Y-origins and wheel mesh diameter) — not invented.
- The wheel velocity/effort bounds (±30 rad/s, 20 N·m) reuse the limits already in the
  URDF from the SolidWorks export.
- Teleop speed (0.3 m/s linear, 0.8 rad/s angular) and the controller's
  `max_velocity`/`max_acceleration` limits are **operating defaults for testing**, not
  CAD-derived — raise them once real motor torque/RPM and gear ratio are known.
- No PID gains, motor torque curve, gear ratio, or encoder resolution have been
  invented anywhere in this phase, per the project's own rule of not fabricating
  mechanical/electrical values that should come from the actual hardware spec.

## Multi-robot warehouse fleet simulation

**See `QUICKSTART.md` for the fastest path to a running fleet, and `TROUBLESHOOTING.md`
for every bug already hit and fixed while building this** — check that file before
re-debugging something that may already have a documented root cause.

Extends the single-robot setup above into a multi-robot warehouse fleet: a 15x20m
warehouse (`worlds/warehouse_fleet_world.sdf`, resized down from the original 50x50m spec
for lighter local testing - regenerate with a larger footprint to scale back up), input
stations along the west wall (x=-6.9) and output stations along the east wall (x=+6.9),
10 per side, 2m apart. N robots each run their own full SLAM Toolbox + Nav2 stack, and a
fleet task manager (`scripts/fleet_manager.py`) continuously assigns idle robots random
pickup/dropoff runs between the two sides.

```bash
ros2 launch amr_ros_dg fleet.launch.py num_robots:=3
```

`num_robots` defaults to 5. Two performance-related launch args, both opt-in and both
independent of simulation logic (see `TROUBLESHOOTING.md` #23 for why they exist):

| Argument | Default | Effect |
|---|---|---|
| `headless` | `true` | Runs Gazebo Sim without its GUI window (`-s`). The web dashboard's camera/lidar views work either way; pass `headless:=false` to see the 3D view directly. |
| `enable_cameras` | `true` | Pass `false` to skip the GPU-rendered d435i/floor cameras on every robot (Nav2/SLAM only use the lidar) — cuts render cost when running several robots with `headless:=false`. |

The launch file's structure (one `GroupAction` per robot, built in a loop) is meant to
scale well past a handful of robots without a rewrite, though very high counts (15-20)
have not been extensively performance-tested — see "Scaling considerations" below and
`TROUBLESHOOTING.md` #21-23 for the concrete bottlenecks found so far (controller-spawner
timeouts under load, costmap cell-count cost, WSL2 GPU render overhead).

### Web dashboard (multi-robot)

`scripts/web_dashboard.py` supports watching/driving any robot in a fleet run, switchable
live from a dropdown in the page itself (no restart needed to change robots):

```bash
ROBOT_LIST=robot1,robot2,robot3 ros2 run amr_ros_dg web_dashboard.py
```

Then open `http://localhost:8080`. Shows that robot's D435i camera, floor camera, a
top-down rendering of its lidar scan, the 12x ultrasonic ring, wheel telemetry, and a
manual drive override (bypasses whatever Nav2 goal is active while a key/button is held).
`ROBOT_NS=robot1` (singular) still works as shorthand for a one-robot dashboard, matching
the single-robot section above.

### Architecture

- **Per-robot, not centralized.** Each `robotN` gets its own `robot_state_publisher`,
  `joint_state_broadcaster` + `diff_drive_controller`, sensor bridge, `slam_toolbox`
  (async), and `nav2_bringup` `navigation_launch.py` (not `bringup_launch.py` - SLAM
  Toolbox supplies the map, so `map_server`/`amcl` aren't needed). There is no shared map
  and no centralized multi-agent planner.
- **No-collision** is satisfied by each robot's own Nav2 local costmap being populated
  from its own lidar (`urdf/amr_ros_dg.urdf.xacro`'s new `lidar_link`, a full 360deg
  gpu_lidar - see the sensors table below) - not by any coordination between robots.
- **Namespacing vs. string-prefixing (the core multi-robot design decision):** Gazebo
  entity/link/sensor-topic uniqueness (needed because gz-transport topics and spawned
  model names are global to the world) is handled by (a) a distinct `-name robotN` on each
  `ros_gz_sim create` spawn call, and (b) the xacro's `robot_name` arg, which prefixes only
  its gz-sim sensor `<topic>` overrides. ROS-side topic/TF uniqueness is handled by ROS 2
  **namespacing** instead - every node for a robot runs inside a `robotN` namespace, which
  routes its TF onto `/robotN/tf`. Frame IDs themselves stay plain (`map`/`odom`/
  `base_link`) for every robot; namespaced `/tf` topics are what actually keep the TF
  trees separate, which is the standard documented multi-robot Nav2 pattern (see the
  turtlebot3 multi-robot demo). Full reasoning is in the comment blocks at the top of
  `urdf/amr_ros_dg.urdf.xacro` and `launch/fleet.launch.py`.
- **Station reservation.** `fleet_manager.py` keeps a single `dict` of
  `station_name -> claiming_robot`, guarded by a `threading.Lock` (needed because the node
  runs a `MultiThreadedExecutor` so per-robot callback groups genuinely run concurrently),
  so two robots can never be assigned the same station at the same time. This is the
  *only* cross-robot coordination in the whole system - motion/collision avoidance is
  entirely per-robot (local costmap + Nav2), per spec.
- **Motion profile.** `config/nav2_params.yaml`'s `velocity_smoother` uses a low
  `max_accel`/`max_decel` relative to `max_velocity` (2 m/s cruise speed, 0.6/1.6 m/s^2
  accel/decel) so the spec'd slow-start / faster-cruise / decel-to-stop profile is visibly
  a ramp, not an instant jump - configuration only, no hand-rolled trapezoidal controller.
  The wheel joints' own velocity command-interface limit (`urdf/amr_ros_dg.urdf.xacro`,
  60 rad/s) sits comfortably above what this cruise speed ever demands (2 m/s / 0.075m
  wheel radius = 26.7 rad/s) - keep this headroom if raising speeds further.
- **Planner.** `nav2_params.yaml` uses NavFn as-is (`nav2_navfn_planner::NavfnPlanner`,
  default tuning) for "fastest route", per spec.

### New sensor: 2D SLAM lidar

| Link | Type | Mount | Placement basis |
|---|---|---|---|
| `lidar_link` | 2D planar lidar, 120 samples, 120deg forward-facing sweep (+-60deg), 0.1-12m | Top of chassis, near-flush, fixed | x=0.32m fwd, z=0.002m - see below for why both the angle and the flush mount matter together |

This is the sensor SLAM Toolbox and each robot's Nav2 costmaps actually consume
(`scan` topic, bridged per-robot as `/robotN/scan`) - distinct from the pre-existing
single-ray ultrasonic ring, which stays as harmless extra sensors (see below).

**Narrowed from a full 360deg sweep to 120deg forward-only** on purpose, not as a
simplification: a 360deg sweep mounted flush against/inside the chassis surface causes
the robot to detect its own body at near-zero range (gz-sim's `gpu_lidar` ray-casts
against the rendered mesh, not collision geometry), which made `collision_monitor`
permanently withhold `cmd_vel` thinking a collision was imminent. Since Nav2 drives this
fleet forward-only, a 120deg forward cone gives full navigation coverage with nothing of
the robot's own body inside it - see `TROUBLESHOOTING.md` #19/#20 for the full story.
This does mean the robot is effectively blind to its sides/rear for SLAM purposes; that's
an accepted tradeoff here, not an oversight.

### Known scaling considerations

- Not yet performance-tested at very high robot counts (15-20) - the architecture
  (per-robot namespacing, loop-built launch actions) is designed to scale; actual CPU/GPU
  headroom on any given machine is unverified past what's been tested locally (up to 3
  robots with the Gazebo GUI + cameras on, more with `headless:=true enable_cameras:=false`).
- N independent SLAM Toolbox + Nav2 stacks (each with its own costmaps, planner, and
  controller running at their configured frequencies) is the dominant CPU cost and scales
  ~linearly with robot count. `TROUBLESHOOTING.md` #22 covers a real costmap cell-count
  regression hit while sizing the global costmap for this world, and the frequency
  tuning applied to compensate.
- Every robot also renders a GPU lidar plus (if `enable_cameras:=true`, the default) a
  D435i camera, floor camera, and 12 single-ray ultrasonic `gpu_lidar` sensors - GPU
  rendering cost, not just CPU, becomes the likely bottleneck before CPU does. Pass
  `enable_cameras:=false` to drop the two RGB cameras (lidar/ultrasonics unaffected) -
  see `TROUBLESHOOTING.md` #23 for the measured impact of this plus `headless:=true`.
- `controller_manager` spawners can time out under heavy simultaneous load at higher
  robot counts (`TROUBLESHOOTING.md` #21) - `fleet.launch.py` already raises
  `--controller-manager-timeout` to 60s to absorb this rather than fail.
- `fleet.launch.py` staggers robot spawns 0.5s apart and waits for each robot's
  `diff_drive_controller` spawner to actually exit (not a fixed timer) before starting
  that robot's SLAM/Nav2 - a real readiness check, not a heuristic delay, specifically
  because the fixed-delay version broke under real fleet load (see
  `TROUBLESHOOTING.md` for the general pattern of preferring event-driven waits here).

## Mesh/unit history (why `legacy/` exists)

The SolidWorks STL export needed three passes to get right:

1. **First export** — every link's STL accidentally contained the *entire assembly*
   (the exporter failed to isolate each component before saving), so every link
   rendered the whole robot on top of itself in RViz.
2. **Second export** — components were correctly isolated, but `base_link.STL` was
   exported outside its own coordinate system (offset ~80cm from where it should be),
   and all meshes came out in millimeters instead of meters. A `scale="0.001 0.001 0.001"`
   workaround was used temporarily; that mesh set is kept at
   `legacy/broken_export_2_mm_scale_bad_baselink_origin/`.
3. **Third export (current)** — a full assembly re-export produced correctly isolated,
   correctly scaled (meters), correctly centered meshes for all 11 links. One CSV glitch
   in this pass (`caster_b_l` mass/inertia read back as all zeros) was caught and is
   **not** used — `caster_b_l` in the xacro keeps its valid values from the first export.

See `legacy/README.md` for the full breakdown of what's kept and why.

## Known limitations / next steps

- No IMU yet (physically exists on the robot but isn't in the URDF/SDF). A 2D planar
  lidar (`lidar_link`, topic `scan`) was added for the multi-robot fleet simulation (see
  "Multi-robot warehouse fleet simulation" below) - SLAM Toolbox and Nav2 consume it
  directly; no IMU fusion is used yet, odometry is wheel-only (see next point).
- `diff_drive_controller` odometry is wheel-only (no IMU fusion) - fine for teleop
  testing, not accurate enough for autonomous navigation yet.
- The two caster-wheel rolling axes (`caster_wheel_f_l_joint`, `caster_wheel_b_l_joint`)
  are slightly off-axis from pure X (a few degrees) — this reflects the actual CAD
  mate geometry (caster trail offset) and was not simplified/rounded.
- Package name follows ROS 2 / REP 144 convention (`amr_ros_dg`, lowercase +
  underscores); the robot's marketing/CAD name (`AMR-ROS-DG`) is kept in prose and
  file/folder names outside the package itself.

# Quick start — multi-robot warehouse fleet

Fastest path from a clean WSL2/Ubuntu terminal to a working fleet simulation. For the
single-robot teleop/Gazebo path, see the main `README.md` instead. If something breaks,
check `TROUBLESHOOTING.md` first — most failure modes here have already been hit and
fixed once.

## 0. One-time setup

Symlink the package into your colcon workspace instead of copying it — edits made on the
Windows side then apply immediately, no manual re-copy step, ever:

```bash
mkdir -p ~/ros2_ws/src
ln -s "/path/to/warehouse-amr-emergent-agents/src/amr_ros_dg" ~/ros2_ws/src/amr_ros_dg
ls -la ~/ros2_ws/src/amr_ros_dg   # confirm it shows "-> /path/to/..."
```

(Optional, only if the simulation feels slow) Give WSL2 more resources — create
`C:\Users\<you>\.wslconfig` on the Windows side:

```ini
[wsl2]
memory=24GB
processors=12
swap=8GB
```

Then from PowerShell (not WSL): `wsl --shutdown`, then reopen a WSL terminal. Also worth
running `wsl --update` once — see `TROUBLESHOOTING.md` #23 for the measured impact.

## 1. Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select amr_ros_dg
source install/setup.bash
```

## 2. Launch the fleet

```bash
ros2 launch amr_ros_dg fleet.launch.py num_robots:=3
```

Useful launch arguments (all optional, all combinable):

| Argument | Default | What it does |
|---|---|---|
| `num_robots` | `5` | How many robots (`robot1`..`robotN`) to spawn |
| `headless` | `true` | `false` opens the Gazebo GUI window (costs real CPU — see below) |
| `enable_cameras` | `true` | `false` skips the GPU-rendered d435i/floor cameras (lidar/Nav2 unaffected) |
| `world` | the bundled 15x20m warehouse | path to a different `.sdf` world |

Each robot spawns with its own full SLAM Toolbox + Nav2 stack, staggered a fraction of a
second apart. `fleet_manager.py` starts assigning random pickup→dropoff tasks
automatically after a fixed startup delay — you don't need to send any goals manually.

## 3. Watch it work

**Terminal 2** — the web dashboard (camera + lidar view + telemetry + manual drive
override, works for any robot, switchable live from a dropdown, no restart needed):

```bash
ROBOT_LIST=robot1,robot2,robot3 ros2 run amr_ros_dg web_dashboard.py
```

Open `http://localhost:8080` in a browser (works from Windows directly — WSL2 forwards
the port automatically).

**Terminal 3** — raw task/log activity:

```bash
grep -n "assigned\|arrived\|done at\|rejected\|failed" ~/ros2_ws/fleet.log 2>/dev/null \
  || echo "tip: pipe your launch command through 'tee ~/ros2_ws/fleet.log' to enable this"
```

Recommended: always run the launch command through `tee` so you have a log file to grep
against without re-running anything:

```bash
ros2 launch amr_ros_dg fleet.launch.py num_robots:=3 2>&1 | tee ~/fleet.log
```

## 4. Sanity checks if something looks wrong

Is the simulation actually keeping up with real time (the #1 root cause of "robots move
so slowly" — see `TROUBLESHOOTING.md` #22/#23)?

```bash
timeout 6 ros2 topic hz /clock
```
A healthy, lightly-loaded run should show a fairly steady rate with a low std-dev. A
low/bursty rate (e.g. single-digit Hz with multi-second gaps) means the SIMULATION is
running slower than real time — not a code bug — see `TROUBLESHOOTING.md` #22/#23 for
what to try (fewer robots, `headless:=true`, `enable_cameras:=false`, `.wslconfig`).

Is a specific robot's wheel actually receiving commands?

```bash
timeout 8 ros2 topic echo /robot1/diff_drive_controller/odom --field pose.pose.position
ros2 control list_controllers --controller-manager /robot1/controller_manager
```

Is CPU/RAM the bottleneck right now?

```bash
free -h
top -bn1 | head -15
```

## 5. Scaling up

Start at `num_robots:=1`, confirm it moves and completes at least one pickup→dropoff
cycle, then step up (`3`, then whatever you actually need). Jumping straight to 10+ makes
every failure ambiguous (bug vs. system load) — see `TROUBLESHOOTING.md` #21-23.

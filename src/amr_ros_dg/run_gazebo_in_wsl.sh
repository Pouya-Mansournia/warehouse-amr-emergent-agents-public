#!/usr/bin/env bash
# One-shot: copy amr_ros_dg into the colcon workspace, build it, and launch it in
# Gazebo Sim with ros2_control (joint_state_broadcaster + diff_drive_controller) active.
#
# Run this INSIDE WSL (Ubuntu 24.04), AFTER run_in_wsl.sh has been run at least once
# (that's what installs ROS 2 + Gazebo Sim + ros2_control packages):
#   bash "/mnt/<drive>/<path-to>/warehouse-amr-ros2/ros2-package/amr_ros_dg/run_gazebo_in_wsl.sh"
#
# Optionally pass the xacro filename (from urdf/) to launch instead of the real robot, e.g.
# to run the no-caster/no-sensor diagnostic robot:
#   bash ".../run_gazebo_in_wsl.sh" test_no_casters.urdf.xacro
#
# Once Gazebo is up and the robot is spawned, drive it from a SECOND WSL terminal with:
#   source ~/ros2_ws/install/setup.bash
#   ros2 run amr_ros_dg wasd_teleop.py

set -e

# Locate the package relative to this script's own path instead of hardcoding a
# Windows location - so it keeps working no matter where the project folder lives
# or gets moved/cloned to (e.g. after moving from OneDrive to D:\...).
WIN_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$HOME/ros2_ws"
DISTRO="jazzy"
MODEL_FILE="${1:-amr_ros_dg.urdf.xacro}"

source "/opt/ros/${DISTRO}/setup.bash"

echo "==> 1/3  Copying package from Windows into WSL workspace..."
mkdir -p "$WS/src"
rm -rf "$WS/src/amr_ros_dg"
cp -r "$WIN_SRC" "$WS/src/amr_ros_dg"

echo "==> 2/3  Building with colcon..."
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y || true
colcon build --packages-select amr_ros_dg --symlink-install

echo "==> 3/3  Launching Gazebo Sim + ros2_control (model: $MODEL_FILE)..."
source "$WS/install/setup.bash"
ros2 launch amr_ros_dg gazebo.launch.py \
  model:="$WS/install/amr_ros_dg/share/amr_ros_dg/urdf/$MODEL_FILE"

#!/usr/bin/env bash
# One-shot: install ROS 2 Jazzy if missing, copy amr_ros_dg into a colcon workspace,
# build it, and launch RViz2 with the robot model + joint sliders.
#
# Run this INSIDE WSL (Ubuntu 24.04):
#   bash "/mnt/<drive>/<path-to>/warehouse-amr-ros2/ros2-package/amr_ros_dg/run_in_wsl.sh"

set -e

# Locate the package relative to this script's own path instead of hardcoding a
# Windows location - so it keeps working no matter where the project folder lives
# or gets moved/cloned to (e.g. after moving from OneDrive to D:\...).
WIN_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$HOME/ros2_ws"
DISTRO="jazzy"

echo "==> 1/6  Checking ROS 2 apt repo..."
if [ ! -f "/opt/ros/${DISTRO}/setup.bash" ]; then
  echo "    ros-${DISTRO} not found, adding the ROS 2 apt repo (needs sudo password)..."
  sudo apt update
  sudo apt install -y curl gnupg lsb-release
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list
  sudo rosdep init 2>/dev/null || true
else
  echo "    found /opt/ros/${DISTRO}"
fi

# Always (re-)ensure every package this project needs is installed, even if ROS itself was
# already set up in a previous run - apt is idempotent (already-installed packages are a
# no-op), and installing packages one at a time means a single missing/renamed package name
# can't silently block all the others (a plain "apt install -y pkg1 pkg2 ... pkgN" aborts the
# WHOLE transaction, installing NOTHING, if even one name is wrong).
echo "==> Installing/verifying required apt packages..."
sudo apt update
FAILED_PKGS=()
for pkg in \
  ros-${DISTRO}-desktop \
  ros-${DISTRO}-joint-state-publisher-gui \
  ros-${DISTRO}-xacro \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-${DISTRO}-ros-gz-sim \
  ros-${DISTRO}-ros-gz-bridge \
  ros-${DISTRO}-gz-ros2-control \
  ros-${DISTRO}-controller-manager \
  ros-${DISTRO}-joint-state-broadcaster \
  ros-${DISTRO}-diff-drive-controller \
  ; do
  echo "    - $pkg"
  sudo apt install -y "$pkg" || FAILED_PKGS+=("$pkg")
done
rosdep update || true

if [ ${#FAILED_PKGS[@]} -ne 0 ]; then
  echo
  echo "!!  These packages failed to install and were SKIPPED (Gazebo/ros2_control features"
  echo "    that depend on them won't work until you fix this):"
  printf '      - %s\n' "${FAILED_PKGS[@]}"
  echo "    Likely cause: package name changed or isn't released as a binary for ${DISTRO} yet."
  echo "    Copy the exact 'apt install' error above and send it back for a fix."
  echo
fi

source "/opt/ros/${DISTRO}/setup.bash"

echo "==> 2/6  Copying package from Windows into WSL workspace..."
mkdir -p "$WS/src"
rm -rf "$WS/src/amr_ros_dg"
cp -r "$WIN_SRC" "$WS/src/amr_ros_dg"

echo "==> 3/6  Installing rosdep dependencies..."
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y || true

echo "==> 4/6  Building with colcon..."
colcon build --packages-select amr_ros_dg --symlink-install

echo "==> 5/6  Launching RViz2..."
source "$WS/install/setup.bash"
ros2 launch amr_ros_dg display.launch.py

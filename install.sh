#!/usr/bin/env bash
# install.sh — one-shot setup for the 2.5D LiDAR Mapper ROS2 package.
# Run this once after cloning the repo. Requires Ubuntu 22.04 + ROS2 Humble
# already installed (this script does not install ROS2 itself).
#
# Usage:
#   chmod +x install.sh
#   ./install.sh

set -e  # stop on first error, so a failed step doesn't silently continue

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$REPO_DIR/ros2_ws"

echo "=== 2.5D LiDAR Mapper — install ==="
echo "Repo directory: $REPO_DIR"
echo

# ---------- 1. Check ROS2 is installed ----------
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ERROR: ROS2 Humble not found at /opt/ros/humble/setup.bash"
  echo "Install ROS2 Humble first: https://docs.ros.org/en/humble/Installation.html"
  exit 1
fi
echo "[1/5] ROS2 Humble found."
source /opt/ros/humble/setup.bash

# ---------- 2. Check colcon is available ----------
if ! command -v colcon &> /dev/null; then
  echo "[2/5] colcon not found — installing python3-colcon-common-extensions..."
  sudo apt update
  sudo apt install -y python3-colcon-common-extensions
else
  echo "[2/5] colcon already installed."
fi

# ---------- 3. Install ROS2 package dependencies (apt) ----------
echo "[3/5] Installing ROS2 dependencies (robot_state_publisher, laser_geometry, tf2 tools)..."
sudo apt update
sudo apt install -y \
  ros-humble-robot-state-publisher \
  ros-humble-laser-geometry \
  ros-humble-tf2-ros \
  ros-humble-tf2-sensor-msgs \
  ros-humble-tf2-tools

# ---------- 4. Install Python dependencies (pip) ----------
echo "[4/5] Installing Python dependencies..."
pip3 install --user "numpy<2" open3d requests
# NOTE: numpy is pinned below 2.0 because the system scipy/sklearn (used
# internally by Open3D for point cloud cleanup) is compiled against
# NumPy 1.x and breaks under NumPy 2.x.

# ---------- 5. Build the workspace ----------
echo "[5/5] Building the ROS2 workspace..."
cd "$WS_DIR"
colcon build --packages-select lidar_mapper_ros2

echo
echo "=== Build complete ==="
echo
echo "Before running anything, source the workspace in your current shell:"
echo "  source $WS_DIR/install/setup.bash    # bash"
echo "  source $WS_DIR/install/setup.zsh     # zsh"
echo
echo "Then launch:"
echo "  ros2 launch lidar_mapper_ros2 lidar_mapper.launch.py mode:=wireless"
echo
echo "Note: this script does NOT flash the ESP32. Flash esp32_firmware/ separately"
echo "via the Arduino IDE — see README.md for details."

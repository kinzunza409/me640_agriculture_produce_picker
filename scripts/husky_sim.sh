#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SETUP_PATH="${CLEARPATH_SETUP_PATH:-$HOME/clearpath}"
SETUP_PATH="${SETUP_PATH%/}/"
ROBOT_CONFIG="${HUSKY_ROBOT_CONFIG:-$REPO_ROOT/config/husky/a200_default.yaml}"

mkdir -p "$SETUP_PATH"
cp "$ROBOT_CONFIG" "$SETUP_PATH/robot.yaml"

source /opt/ros/humble/setup.bash
if [ -f "$REPO_ROOT/ros_ws/install/setup.bash" ]; then
  source "$REPO_ROOT/ros_ws/install/setup.bash"
fi

exec ros2 launch clearpath_gz simulation.launch.py setup_path:="$SETUP_PATH" rviz:="${RVIZ:-true}" "$@"

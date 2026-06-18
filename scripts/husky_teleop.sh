#!/usr/bin/env bash
set -euo pipefail

NAMESPACE="${HUSKY_NAMESPACE:-/a200_0000}"

source /opt/ros/humble/setup.bash
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r "__ns:=$NAMESPACE"

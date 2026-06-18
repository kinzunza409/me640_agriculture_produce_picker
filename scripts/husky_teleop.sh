#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r "__ns:=${1:-/a200_0000}"
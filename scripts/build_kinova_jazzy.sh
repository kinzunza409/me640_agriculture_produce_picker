#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
WORKSPACE="${PROJECT_ROOT}/ros_ws"
API_SOURCE_DIR="${WORKSPACE}/vendor/kortex_api_2.8.0"
DRIVER_SOURCE_DIR="${WORKSPACE}/vendor/kortex_driver_2.5.0"

if [[ ! -d "${API_SOURCE_DIR}/include" || ! -d "${DRIVER_SOURCE_DIR}/lib" ]]; then
  echo "Prepared Kortex artifacts were not found." >&2
  echo "Run scripts/setup_kinova_jazzy_dependencies.sh first." >&2
  exit 1
fi

cd "${WORKSPACE}"

colcon --log-base log_jazzy build \
  --build-base build_jazzy \
  --install-base install_jazzy \
  --packages-select kortex_api \
  --cmake-args \
    -Wno-dev \
    -DFETCHCONTENT_SOURCE_DIR_KINOVA_BINARY_API="${API_SOURCE_DIR}"

source "${WORKSPACE}/install_jazzy/setup.bash"

colcon --log-base log_jazzy build \
  --build-base build_jazzy \
  --install-base install_jazzy \
  --packages-select kortex_driver \
  --cmake-args \
    -Wno-dev \
    -DFETCHCONTENT_SOURCE_DIR_KINOVA_BINARY_API="${DRIVER_SOURCE_DIR}"

source "${WORKSPACE}/install_jazzy/setup.bash"

colcon --log-base log_jazzy build \
  --build-base build_jazzy \
  --install-base install_jazzy \
  --packages-select kinova_pose_goal \
  --packages-ignore \
    kortex_description \
    kinova_gen3_7dof_robotiq_2f_85_moveit_config

echo "Built kortex_api, kortex_driver, and kinova_pose_goal in install_jazzy."

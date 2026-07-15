#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

IMAGE="${HUSKY_JAZZY_IMAGE:-me640-jazzy-minimal:husky3}"
CONTAINER="${HUSKY_JAZZY_CONTAINER:-husky-jazzy-deploy}"
SERIAL_DEVICE="${HUSKY_SERIAL_DEVICE:-/dev/clearpath/prolific}"
CAMERA_DEVICE="${HUSKY_CAMERA_DEVICE:-}"

echo "Building ${IMAGE} from .devcontainer/jazzy-minimal/Dockerfile..."
docker build \
  -f "$REPO_ROOT/.devcontainer/jazzy-minimal/Dockerfile" \
  -t "$IMAGE" \
  "$REPO_ROOT"

DEVICE_ARGS=(--device="$SERIAL_DEVICE")
if [ -n "$CAMERA_DEVICE" ]; then
  DEVICE_ARGS+=(--device="$CAMERA_DEVICE")
fi

docker rm -f "$CONTAINER" >/dev/null 2>&1 || true

echo "Starting ${CONTAINER}..."
docker run -d \
  --name "$CONTAINER" \
  --hostname=ros-jazzy \
  --network=host \
  --ipc=host \
  "${DEVICE_ARGS[@]}" \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  -e 'ROS_DISCOVERY_SERVER=127.0.0.1:11811;' \
  -e ROS_SUPER_CLIENT=True \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -v "$REPO_ROOT":/project \
  -w /project \
  "$IMAGE" \
  tail -f /dev/null

echo "Building ros_ws (colcon build)..."
docker exec "$CONTAINER" bash -c '
  set -euo pipefail
  source /opt/ros/jazzy/setup.bash
  cd /project/ros_ws
  colcon build --symlink-install
'

echo "Build succeeded. Entering ${CONTAINER}..."
exec docker exec -it "$CONTAINER" bash

#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

IMAGE="${HUSKY_JAZZY_IMAGE:-me640-jazzy-minimal:husky3}"
CONTAINER="${HUSKY_JAZZY_CONTAINER:-husky-jazzy-deploy}"
SERIAL_DEVICE="${HUSKY_SERIAL_DEVICE:-/dev/clearpath/prolific}"

# The D435i (via librealsense's V4L2 backend) needs simultaneous access to
# multiple host device nodes -- /dev/bus/usb/*, several /dev/video*, and several
# /dev/media* -- whose exact numbers are NOT stable across reboots/replugs. A
# single fixed --device= path is not robust, so we bind-mount the whole /dev
# tree instead (proven working on the dev workstation). Set HUSKY_DEV_MOUNT to a
# narrower path to scope it down if you know exactly which nodes are needed.
#
# PREREQUISITE (host-side, NOT handled by this script): mounting /dev only makes
# the nodes visible in the container -- it does not grant permission to open
# them. On the dev workstation the camera's /dev/bus/usb, /dev/video*, and
# /dev/media* nodes needed a udev rule matching idVendor==8086, idProduct==0b3a
# with MODE="0666" before the container could use them. The Husky's onboard
# computer will likely need the same udev rule installed on its host OS
# (requires SSH access + sudo on the robot).
DEV_MOUNT="${HUSKY_DEV_MOUNT:-/dev}"

echo "Building ${IMAGE} from .devcontainer/jazzy-minimal/Dockerfile..."
docker build \
  -f "$REPO_ROOT/.devcontainer/jazzy-minimal/Dockerfile" \
  -t "$IMAGE" \
  "$REPO_ROOT"

# Serial link to the Husky's own hardware stays a narrow --device passthrough.
# Camera nodes come in via the /dev bind mount below (docker --device cannot
# take a directory, so we use -v for the tree).
DEVICE_ARGS=(--device="$SERIAL_DEVICE")

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
  -v "$DEV_MOUNT":/dev \
  -v "$REPO_ROOT":/project \
  -w /project \
  "$IMAGE" \
  tail -f /dev/null

echo "Entering ${CONTAINER}..."
exec docker exec -it "$CONTAINER" bash

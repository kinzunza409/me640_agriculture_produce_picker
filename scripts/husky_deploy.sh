#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

IMAGE="${HUSKY_JAZZY_IMAGE:-me640-jazzy-minimal:husky3}"
CONTAINER="${HUSKY_JAZZY_CONTAINER:-husky-jazzy-deploy}"
SERIAL_DEVICE="${HUSKY_SERIAL_DEVICE:-/dev/clearpath/prolific}"

# jazzy-minimal's librealsense is built with FORCE_RSUSB_BACKEND=ON (see
# .devcontainer/jazzy-minimal/Dockerfile), so the D435i is only ever accessed
# via /dev/bus/usb/* (libusb) -- no /dev/video*/dev/media* V4L2 nodes needed.
# Node numbers under /dev/bus/usb are NOT stable across reboots/replugs, so we
# bind-mount the whole /dev tree rather than a fixed path. Set HUSKY_DEV_MOUNT
# to a narrower path to scope it down if you know exactly which nodes are needed.
DEV_MOUNT="${HUSKY_DEV_MOUNT:-/dev}"

REALSENSE_UDEV_RULE_FILE="/etc/udev/rules.d/99-realsense-d435i.rules"
REALSENSE_UDEV_RULE_CONTENT='SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b3a", MODE="0666"'

# Installs a MODE=0666 udev rule for the D435i's USB device nodes and
# reloads/triggers udev so it applies immediately without a reboot/replug.
# Not strictly required here since the container runs as true root with no
# --userns remap (root already bypasses DAC checks) -- the actual blocker for
# real Docker is the cgroup device rule below -- but this keeps host-side
# permissions correct for any other (non-root) tooling that touches the
# camera outside the container, and matches what linux-gpu's Podman setup
# needed on the dev workstation.
install_realsense_udev_rule() {
  if [ -f "$REALSENSE_UDEV_RULE_FILE" ] && grep -qF "$REALSENSE_UDEV_RULE_CONTENT" "$REALSENSE_UDEV_RULE_FILE"; then
    return
  fi
  echo "Installing RealSense udev rule at ${REALSENSE_UDEV_RULE_FILE}..."
  echo "$REALSENSE_UDEV_RULE_CONTENT" | sudo tee "$REALSENSE_UDEV_RULE_FILE" >/dev/null
  sudo udevadm control --reload-rules
  sudo udevadm trigger --subsystem-match=usb
}

install_realsense_udev_rule

echo "Building ${IMAGE} from .devcontainer/jazzy-minimal/Dockerfile..."
docker build \
  -f "$REPO_ROOT/.devcontainer/jazzy-minimal/Dockerfile" \
  -t "$IMAGE" \
  "$REPO_ROOT"

# Serial link to the Husky's own hardware stays a narrow --device passthrough.
# Camera nodes come in via the /dev bind mount below (docker --device cannot
# take a directory, so we use -v for the tree).
#
# The /dev bind mount alone only makes the device nodes VISIBLE in the
# container -- real Docker (unlike rootless Podman, see linux-gpu's
# devcontainer.json comments) still enforces the cgroup device allowlist, so
# open() on them is denied without an explicit grant. The RSUSB-backend
# camera build only talks to the D435i over /dev/bus/usb/*, which are char
# devices under major 189 (USB_DEVICE_MAJOR), so grant that major here.
DEVICE_ARGS=(--device="$SERIAL_DEVICE" --device-cgroup-rule='c 189:* rmw')

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

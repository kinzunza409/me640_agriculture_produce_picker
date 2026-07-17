#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
WORKSPACE="${PROJECT_ROOT}/ros_ws"
SOURCE_DIR="${WORKSPACE}/src"
VENDOR_DIR="${WORKSPACE}/vendor"
KORTEX_DIR="${SOURCE_DIR}/ros2_kortex_official"
KORTEX_COMMIT=63944dee465d836ea714a114ea3657dfa1617d95

API_ARCHIVE="${VENDOR_DIR}/kortex_api_2.8.0.zip"
API_URL=https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.8.0/linux_x86-64_gcc_5.4.zip
API_MD5=b9b68885df8e1dd0e5a2ba7285f14c0d
API_SOURCE_DIR="${VENDOR_DIR}/kortex_api_2.8.0"

DRIVER_ARCHIVE="${VENDOR_DIR}/kortex_driver_2.5.0.zip"
DRIVER_URL=https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.5.0/linux_x86-64_x86_gcc.zip
DRIVER_MD5=64bd86e7ab8bda90ef1fc7d6a356e080
DRIVER_SOURCE_DIR="${VENDOR_DIR}/kortex_driver_2.5.0"

download_and_verify() {
  local url=$1
  local expected_md5=$2
  local destination=$3

  if [[ ! -f "${destination}" ]]; then
    curl --fail --location --retry 3 --output "${destination}" "${url}"
  fi
  echo "${expected_md5}  ${destination}" | md5sum --check --status || {
    echo "Checksum mismatch: ${destination}" >&2
    echo "Remove the bad archive explicitly, then run this script again." >&2
    return 1
  }
}

mkdir -p "${SOURCE_DIR}" "${VENDOR_DIR}"

if [[ ! -d "${KORTEX_DIR}/.git" ]]; then
  if [[ -e "${KORTEX_DIR}" ]]; then
    echo "${KORTEX_DIR} exists but is not a Git checkout; refusing to replace it." >&2
    exit 1
  fi
  vcs import "${SOURCE_DIR}" < "${WORKSPACE}/kinova_jazzy.repos"
fi

actual_commit=$(git -C "${KORTEX_DIR}" rev-parse HEAD)
if [[ "${actual_commit}" != "${KORTEX_COMMIT}" ]]; then
  echo "ros2_kortex commit mismatch." >&2
  echo "Expected: ${KORTEX_COMMIT}" >&2
  echo "Actual:   ${actual_commit}" >&2
  echo "This script will not checkout or reset an existing repository." >&2
  exit 1
fi

download_and_verify "${API_URL}" "${API_MD5}" "${API_ARCHIVE}"
download_and_verify "${DRIVER_URL}" "${DRIVER_MD5}" "${DRIVER_ARCHIVE}"

mkdir -p "${API_SOURCE_DIR}" "${DRIVER_SOURCE_DIR}"
unzip -q -o "${API_ARCHIVE}" -d "${API_SOURCE_DIR}"
unzip -q -o "${DRIVER_ARCHIVE}" -d "${DRIVER_SOURCE_DIR}"

PATCH_FILE="${PROJECT_ROOT}/patches/ros2_kortex-jazzy-robotiq-xacro.patch"
XACRO_FILE="${KORTEX_DIR}/kortex_description/grippers/robotiq_2f_85/urdf/robotiq_2f_85_macro.xacro"
sed -i 's/\r$//' "${XACRO_FILE}"
if git -C "${KORTEX_DIR}" apply --reverse --check "${PATCH_FILE}" 2>/dev/null; then
  echo "Kortex Jazzy/Robotiq patch already applied."
elif git -C "${KORTEX_DIR}" apply --check "${PATCH_FILE}"; then
  git -C "${KORTEX_DIR}" apply "${PATCH_FILE}"
  echo "Applied Kortex Jazzy/Robotiq patch."
else
  echo "Kortex patch does not apply cleanly; inspect ${XACRO_FILE}." >&2
  exit 1
fi

echo "Kinova Jazzy dependencies are prepared."
echo "ros2_kortex: ${actual_commit}"
echo "Kortex API source: ${API_SOURCE_DIR}"
echo "Kortex driver source: ${DRIVER_SOURCE_DIR}"

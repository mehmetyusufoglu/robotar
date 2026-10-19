#!/usr/bin/env bash
set -euo pipefail

if (( EUID != 0 )); then
  echo "Run this installer with: sudo bash $0"
  exit 1
fi

source /etc/os-release
if [[ "${ID}" != "ubuntu" || "${VERSION_CODENAME}" != "noble" ]]; then
  echo "This installer requires Ubuntu 24.04 (noble); detected ${PRETTY_NAME}."
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y curl ca-certificates python3

if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ROS_APT_SOURCE_VERSION="$(
    curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])'
  )"
  apt_source_deb="$(mktemp /tmp/ros2-apt-source.XXXXXX)"
  trap 'rm -f "${apt_source_deb}"' EXIT
  curl -fsSL \
    -o "${apt_source_deb}" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
  dpkg -i "${apt_source_deb}"
fi

apt-get update
apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-dev-tools

echo
echo "ROS 2 Jazzy and Gazebo Harmonic packages are installed."
echo "Activate ROS 2 in each terminal with: source /opt/ros/jazzy/setup.bash"

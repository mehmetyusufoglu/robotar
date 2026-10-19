#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
WORKSPACES="${PROJECT_ROOT}/third_party/robot_workspaces"

if (( EUID != 0 )); then
    printf 'Run with sudo: sudo %s\n' "$0" >&2
    exit 1
fi

TARGET_USER="${SUDO_USER:-$(stat -c '%U' "${PROJECT_ROOT}")}"
if [[ -z "${TARGET_USER}" || "${TARGET_USER}" == root ]]; then
    printf 'Run this script with sudo from the regular project account.\n' >&2
    exit 1
fi
TARGET_HOME="$(getent passwd "${TARGET_USER}" | cut -d: -f6)"
if [[ -z "${TARGET_HOME}" ]]; then
    printf 'Cannot determine home directory for %s.\n' "${TARGET_USER}" >&2
    exit 1
fi

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-venv \
    python3.12-venv \
    python3-numpy \
    python3-scipy \
    python3-yaml \
    python3-tk \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-behaviortree-cpp \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-moveit \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-pilz-industrial-motion-planner

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    rosdep init
fi
sudo -u "${TARGET_USER}" -H rosdep update --rosdistro jazzy
rosdep -c "${TARGET_HOME}/.ros/rosdep/sources.cache" install \
    --from-paths \
        "${WORKSPACES}/husarion_ws/src" \
        "${WORKSPACES}/franka_ros2_ws/src" \
        "${WORKSPACES}/combined_ws/src" \
    --ignore-src -r -y --rosdistro jazzy \
    --skip-keys "ament_python libfranka olv_module_descriptions franka_gazebo_bringup gz-msgs10 gz-plugin2 gz-sim8 gz-transport13"
rosdep -c "${TARGET_HOME}/.ros/rosdep/sources.cache" check \
    --from-paths \
        "${WORKSPACES}/husarion_ws/src" \
        "${WORKSPACES}/franka_ros2_ws/src" \
        "${WORKSPACES}/combined_ws/src" \
    --ignore-src --rosdistro jazzy \
    --skip-keys "ament_python libfranka olv_module_descriptions franka_gazebo_bringup gz-msgs10 gz-plugin2 gz-sim8 gz-transport13"

printf 'Dependencies complete. Next run:\n  %s/scripts/tomato_setup_yolo.sh\n  %s/scripts/tomato_build.sh\n' \
    "${PROJECT_ROOT}" "${PROJECT_ROOT}"

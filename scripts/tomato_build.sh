#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/tomato_common.sh"
tomato_enable_logging tomato_build

BUILD_WORKERS="${TOMATO_BUILD_WORKERS:-2}"
if [[ ! "${BUILD_WORKERS}" =~ ^[1-9][0-9]*$ ]]; then
    printf 'TOMATO_BUILD_WORKERS must be a positive integer.\n' >&2
    exit 1
fi

(
    tomato_source_jazzy
    cd "${TOMATO_WORKSPACES}/husarion_ws"
    colcon build --symlink-install --parallel-workers "${BUILD_WORKERS}"
)
(
    tomato_source_jazzy
    tomato_source_file "${TOMATO_WORKSPACES}/husarion_ws/install/setup.bash"
    cd "${TOMATO_WORKSPACES}/franka_ros2_ws"
    colcon build --symlink-install --parallel-workers "${BUILD_WORKERS}" \
        --packages-select franka_description
)
(
    tomato_source_jazzy
    tomato_source_file "${TOMATO_WORKSPACES}/husarion_ws/install/setup.bash"
    tomato_source_file "${TOMATO_WORKSPACES}/franka_ros2_ws/install/setup.bash"
    cd "${TOMATO_WORKSPACES}/combined_ws"
    colcon build --symlink-install --parallel-workers "${BUILD_WORKERS}"
)

printf 'Build complete. Runtime logs will be written to %s.\n' "${TOMATO_LOG_DIR}"

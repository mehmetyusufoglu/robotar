#!/usr/bin/env bash
# Build the Robotar workspace (ws/src: robotar_description, robotar_gazebo,
# robotar_mission) with colcon. Idempotent; ROBOTAR_BUILD_WORKERS controls
# parallelism (default: nproc).
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=robotar_common.sh
source "${SCRIPT_DIR}/robotar_common.sh"

robotar_enable_logging robotar_build
robotar_source_jazzy

WORKERS="${ROBOTAR_BUILD_WORKERS:-$(nproc)}"
printf 'Building %s with %s worker(s)\n' "${ROBOTAR_WS}" "${WORKERS}"
cd "${ROBOTAR_WS}"
colcon build --symlink-install --parallel-workers "${WORKERS}"
printf 'Build finished. Source with: source %s/install/setup.bash\n' "${ROBOTAR_WS}"

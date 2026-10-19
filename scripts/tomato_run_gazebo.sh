#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/tomato_common.sh"
tomato_source_overlays
LOG_NAME=tomato_gazebo
case "${TOMATO_GPU_RENDER:-0}" in
    1|true|TRUE|yes|YES|on|ON) LOG_NAME=tomato_gazebo_gpu ;;
esac
tomato_enable_logging "${LOG_NAME}"
GUI="$(tomato_gui_value)"
tomato_gpu_render_enable

cmd=(
    ros2 launch combined_robot combined_gazebo_sera.launch.py
    "world_file:=${TOMATO_WORLD_FILE}"
    "gz_gui:=${GUI}"
    "run_rviz:=${GUI}"
)
cmd+=("$@")
printf 'Executing:'
printf ' %q' "${cmd[@]}"
printf '\nStop with Ctrl-C.\n'
exec "${cmd[@]}"

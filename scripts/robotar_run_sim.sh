#!/usr/bin/env bash
# Run the Robotar orchard simulation (no mission nodes).
#   ROBOTAR_GUI=1            Gazebo GUI (default headless)
#   ROBOTAR_GPU_RENDER=1     NVIDIA PRIME render offload
#   ROBOTAR_TOOL=comb|gripper|shaker (default comb)
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=robotar_common.sh
source "${SCRIPT_DIR}/robotar_common.sh"

robotar_enable_logging robotar_sim
robotar_source_overlay
robotar_gpu_render_enable

GUI="$(robotar_gui_value)"
TOOL="$(robotar_tool_value)"
printf 'Starting robotar sim (gui=%s tool=%s)\n' "${GUI}" "${TOOL}"
exec ros2 launch robotar_gazebo sim.launch.py gui:="${GUI}" tool:="${TOOL}"

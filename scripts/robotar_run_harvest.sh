#!/usr/bin/env bash
# Run the Robotar olive-harvest demo end to end (sim + mission).
# The harvest mission requires the comb tool; use robotar_run_sim.sh with
# ROBOTAR_TOOL=gripper for the gripper variant.
#   ROBOTAR_GUI=1          Gazebo GUI (default headless)
#   ROBOTAR_GPU_RENDER=1   NVIDIA PRIME render offload
#   ROBOTAR_MISS_CONTACT=1 no-contact negative verification (deliberate miss
#                          pose; no detach is published; mission fails cleanly)
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=robotar_common.sh
source "${SCRIPT_DIR}/robotar_common.sh"

robotar_enable_logging robotar_harvest
robotar_source_overlay
robotar_gpu_render_enable

TOOL="$(robotar_tool_value)"
if [[ "${TOOL}" != "comb" ]]; then
    printf 'robotar_run_harvest.sh requires the comb tool (ROBOTAR_TOOL=%s).\n' "${TOOL}" >&2
    printf 'Use robotar_run_sim.sh for the gripper variant.\n' >&2
    exit 2
fi

GUI="$(robotar_gui_value)"
MISS="$(robotar_miss_contact_value)"
printf 'Starting robotar harvest demo (gui=%s miss_contact=%s)\n' "${GUI}" "${MISS}"
exec ros2 launch robotar_mission harvest_demo.launch.py \
    gui:="${GUI}" miss_contact:="${MISS}"

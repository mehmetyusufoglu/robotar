#!/usr/bin/env bash
# Run the Robotar branch clamp-shaker demo end to end (sim + mission).
# The shaker mission requires the shaker tool; robotar_run_harvest.sh is the
# legacy comb demo, robotar_run_sim.sh covers the bare sim variants.
#   ROBOTAR_GUI=1          Gazebo GUI (default headless)
#   ROBOTAR_GPU_RENDER=1   NVIDIA PRIME render offload
#   ROBOTAR_MISS_CONTACT=1 no-clamp-contact negative verification (deliberate
#                          miss pose; clamp never verifies; no shake, no
#                          detach; mission fails cleanly)
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=robotar_common.sh
source "${SCRIPT_DIR}/robotar_common.sh"

robotar_enable_logging robotar_shaker
robotar_source_overlay
robotar_gpu_render_enable

TOOL="$(robotar_tool_value)"
if [[ "${TOOL}" != "shaker" ]]; then
    printf 'robotar_run_shaker.sh requires the shaker tool (ROBOTAR_TOOL=%s).\n' "${TOOL}" >&2
    printf 'Use robotar_run_harvest.sh for the legacy comb demo.\n' >&2
    exit 2
fi

GUI="$(robotar_gui_value)"
MISS="$(robotar_miss_contact_value)"
printf 'Starting robotar shaker demo (gui=%s miss_contact=%s)\n' "${GUI}" "${MISS}"
exec ros2 launch robotar_mission shaker_demo.launch.py \
    gui:="${GUI}" miss_contact:="${MISS}"

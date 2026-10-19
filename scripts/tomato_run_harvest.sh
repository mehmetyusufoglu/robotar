#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/tomato_common.sh"
tomato_source_overlays

if [[ ! -f "${TOMATO_YOLO_MODEL}" ]]; then
    printf 'Missing YOLO model: %s\n' "${TOMATO_YOLO_MODEL}" >&2
    exit 1
fi
if [[ ! -f "${TOMATO_YOLO_SITE_PACKAGES}/torch/__init__.py" || \
      ! -f "${TOMATO_YOLO_SITE_PACKAGES}/ultralytics/__init__.py" ]]; then
    printf 'Incomplete YOLO environment. Run %s/scripts/tomato_setup_yolo.sh first.\n' \
        "${TOMATO_PROJECT_ROOT}" >&2
    exit 1
fi

LOG_NAME=tomato_harvest
case "${TOMATO_GPU_RENDER:-0}" in
    1|true|TRUE|yes|YES|on|ON) LOG_NAME=tomato_harvest_gpu ;;
esac
if [[ "${TOMATO_DEVICE:-cpu}" != cpu ]]; then
    LOG_NAME=tomato_harvest_gpu
fi
tomato_enable_logging "${LOG_NAME}"
GUI="$(tomato_gui_value)"
YOLO_DEVICE="$(tomato_yolo_device)"
tomato_gpu_render_enable
if [[ "${YOLO_DEVICE}" != cpu ]]; then
    "${TOMATO_YOLO_VENV}/bin/python" -c \
        'import torch; assert torch.cuda.is_available(), "CUDA requested but unavailable"'
fi
cmd=(
    ros2 launch combined_robot sera_spawn_harvest_demo.launch.py
    "world_file:=${TOMATO_WORLD_FILE}"
    mission_autostart:=true
    "yolo_device:=${YOLO_DEVICE}"
    "yolo_model_path:=${TOMATO_YOLO_MODEL}"
    "yolo_site_packages:=${TOMATO_YOLO_SITE_PACKAGES}"
    "run_rviz:=${GUI}"
    "run_gazebo_gui_client:=${GUI}"
    run_yolo_bbox_viewer:=false
    run_tomato_map_panel:=false
)
cmd+=("$@")
printf 'Executing:'
printf ' %q' "${cmd[@]}"
printf '\nStop with Ctrl-C.\n'
exec "${cmd[@]}"

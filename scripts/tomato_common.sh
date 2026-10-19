#!/usr/bin/env bash

TOMATO_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
TOMATO_WORKSPACES="${TOMATO_PROJECT_ROOT}/third_party/robot_workspaces"
TOMATO_YOLO_VENV="${TOMATO_PROJECT_ROOT}/venvs/yolo"
TOMATO_PYTHON_VERSION="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
TOMATO_YOLO_SITE_PACKAGES="${TOMATO_YOLO_VENV}/lib/python${TOMATO_PYTHON_VERSION}/site-packages"
TOMATO_YOLO_MODEL="${TOMATO_WORKSPACES}/combined_ws/yolo_models/tomato/best.pt"
TOMATO_WORLD_FILE="${TOMATO_WORLD_FILE:-tomato_farm_sera_cpu.sdf}"
TOMATO_LOG_DIR="${TOMATO_PROJECT_ROOT}/data/logs"
export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-${TOMATO_LOG_DIR}/tomato_ultralytics}"

tomato_source_file() {
    local setup_file="$1"
    local restore_nounset=false
    if [[ $- == *u* ]]; then
        set +u
        restore_nounset=true
    fi
    # shellcheck disable=SC1090
    source "${setup_file}"
    local source_status=$?
    if [[ "${restore_nounset}" == true ]]; then
        set -u
    fi
    return "${source_status}"
}

tomato_source_jazzy() {
    tomato_source_file /opt/ros/jazzy/setup.bash
}

tomato_source_overlays() {
    local setup_file
    tomato_source_jazzy
    for setup_file in \
        "${TOMATO_WORKSPACES}/husarion_ws/install/setup.bash" \
        "${TOMATO_WORKSPACES}/franka_ros2_ws/install/setup.bash" \
        "${TOMATO_WORKSPACES}/combined_ws/install/setup.bash"; do
        if [[ ! -f "${setup_file}" ]]; then
            printf 'Missing overlay: %s\nRun %s/scripts/tomato_build.sh first.\n' \
                "${setup_file}" "${TOMATO_PROJECT_ROOT}" >&2
            return 1
        fi
        tomato_source_file "${setup_file}"
    done

    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE="${TOMATO_WORKSPACES}/combined_ws/disable_fastdds_shm.xml"
    export RMW_FASTRTPS_USE_QOS_FROM_XML=1
}

tomato_enable_logging() {
    local log_name="$1"
    mkdir -p "${TOMATO_LOG_DIR}" "${YOLO_CONFIG_DIR}/Ultralytics"
    TOMATO_LOG_FILE="${TOMATO_LOG_DIR}/${log_name}_$(date +%Y%m%d_%H%M%S).log"
    exec > >(tee -a "${TOMATO_LOG_FILE}") 2>&1
    printf 'Logging to %s\n' "${TOMATO_LOG_FILE}"
}

tomato_gui_value() {
    case "${TOMATO_GUI:-0}" in
        1|true|TRUE|yes|YES|on|ON) printf 'true\n' ;;
        0|false|FALSE|no|NO|off|OFF) printf 'false\n' ;;
        *)
            printf 'TOMATO_GUI must be 0 or 1, not %s\n' "${TOMATO_GUI}" >&2
            return 1
            ;;
    esac
}

tomato_yolo_device() {
    printf '%s\n' "${TOMATO_DEVICE:-cpu}"
}

tomato_gpu_render_enable() {
    case "${TOMATO_GPU_RENDER:-0}" in
        1|true|TRUE|yes|YES|on|ON)
            export __NV_PRIME_RENDER_OFFLOAD=1
            export __GLX_VENDOR_LIBRARY_NAME=nvidia
            ;;
        0|false|FALSE|no|NO|off|OFF) ;;
        *)
            printf 'TOMATO_GPU_RENDER must be 0 or 1, not %s\n' "${TOMATO_GPU_RENDER}" >&2
            return 1
            ;;
    esac
}

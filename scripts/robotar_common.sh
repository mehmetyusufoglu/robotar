#!/usr/bin/env bash
# Shared helpers for the Robotar entry-point scripts.
#
# Environment contract:
#   ROBOTAR_GUI=0|1          Gazebo GUI (default 0/headless)
#   ROBOTAR_GPU_RENDER=0|1   NVIDIA PRIME render offload (default 0)
#   ROBOTAR_TOOL=comb|gripper  tool mounted by the description (default comb)
#   ROBOTAR_MISS_CONTACT=0|1 harvest no-contact verification (default 0)
#   ROBOTAR_BUILD_WORKERS=N  colcon parallel workers (default: nproc)
#
# Only /opt/ros/jazzy and the project ws/install overlay are sourced.

ROBOTAR_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
ROBOTAR_WS="${ROBOTAR_PROJECT_ROOT}/ws"
ROBOTAR_LOG_DIR="${ROBOTAR_PROJECT_ROOT}/data/logs"

robotar_source_file() {
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

robotar_source_jazzy() {
    robotar_source_file /opt/ros/jazzy/setup.bash
}

robotar_source_overlay() {
    robotar_source_jazzy
    local setup_file="${ROBOTAR_WS}/install/setup.bash"
    if [[ ! -f "${setup_file}" ]]; then
        printf 'Missing overlay: %s\nRun %s/scripts/robotar_build.sh first.\n' \
            "${setup_file}" "${ROBOTAR_PROJECT_ROOT}" >&2
        return 1
    fi
    robotar_source_file "${setup_file}"
}

robotar_enable_logging() {
    local log_name="$1"
    mkdir -p "${ROBOTAR_LOG_DIR}"
    ROBOTAR_LOG_FILE="${ROBOTAR_LOG_DIR}/${log_name}_$(date +%Y%m%d_%H%M%S).log"
    exec > >(tee -a "${ROBOTAR_LOG_FILE}") 2>&1
    printf 'Logging to %s\n' "${ROBOTAR_LOG_FILE}"
}

robotar_bool_value() {
    local name="$1"
    local value="$2"
    case "${value}" in
        1|true|TRUE|yes|YES|on|ON) printf 'true\n' ;;
        0|false|FALSE|no|NO|off|OFF) printf 'false\n' ;;
        *)
            printf '%s must be 0 or 1, not %s\n' "${name}" "${value}" >&2
            return 1
            ;;
    esac
}

robotar_gui_value() {
    robotar_bool_value ROBOTAR_GUI "${ROBOTAR_GUI:-0}"
}

robotar_miss_contact_value() {
    robotar_bool_value ROBOTAR_MISS_CONTACT "${ROBOTAR_MISS_CONTACT:-0}"
}

robotar_tool_value() {
    case "${ROBOTAR_TOOL:-comb}" in
        comb|gripper) printf '%s\n' "${ROBOTAR_TOOL:-comb}" ;;
        *)
            printf 'ROBOTAR_TOOL must be comb or gripper, not %s\n' "${ROBOTAR_TOOL}" >&2
            return 1
            ;;
    esac
}

# NVIDIA PRIME render offload, opt-in via ROBOTAR_GPU_RENDER=1. Kept only in
# these root scripts; launch files stay renderer-agnostic.
robotar_gpu_render_enable() {
    case "${ROBOTAR_GPU_RENDER:-0}" in
        1|true|TRUE|yes|YES|on|ON)
            export __NV_PRIME_RENDER_OFFLOAD=1
            export __GLX_VENDOR_LIBRARY_NAME=nvidia
            ;;
        0|false|FALSE|no|NO|off|OFF) ;;
        *)
            printf 'ROBOTAR_GPU_RENDER must be 0 or 1, not %s\n' "${ROBOTAR_GPU_RENDER}" >&2
            return 1
            ;;
    esac
}

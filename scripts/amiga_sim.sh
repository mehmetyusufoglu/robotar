#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
COMPOSE=(docker compose -f "${PROJECT_ROOT}/docker/amiga/compose.yaml")

case "${1:-gazebo}" in
    gazebo)
        "${COMPOSE[@]}" up -d ros
        "${COMPOSE[@]}" exec -T ros bash -ic '
            exec ros2 launch amiga_ros2_gazebo sim_bringup.launch.py robot_count:=1 headless:=false launch_rviz:=false launch_nav:=false launch_arm:=false launch_helpers:=false launch_bt:=false launch_coordination:=false launch_agents:=false launch_vlm:=false lora_hardware:=false broken_sampler_robot:=0 spawn_person:=0 spawn_truck:=0 remove_tree:=0
        '
        ;;
    arm)
        "${COMPOSE[@]}" exec -T ros bash -ic '
            exec ros2 launch amiga_ros2_gazebo sim_arm.launch.py launch_rviz:=false launch_moveto:=false launch_segment_leaves_sim:=false "sampler_fail_goals:=[]"
        '
        ;;
    *)
        echo "Usage: $0 [gazebo|arm]" >&2
        exit 2
        ;;
esac

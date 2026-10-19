#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
SOURCE_DIR="${PROJECT_ROOT}/third_party/amiga-ros2-bridge"
COMPOSE=(docker compose -f "${PROJECT_ROOT}/docker/amiga/compose.yaml")
EXPECTED_COMMIT="$(
    python3 - "${PROJECT_ROOT}/config/third_party.repos" <<'PY'
import pathlib
import sys
import yaml

config = yaml.safe_load(pathlib.Path(sys.argv[1]).read_text())
print(config["repositories"]["third_party/amiga-ros2-bridge"]["version"])
PY
)"

actual_commit=$(git -C "${SOURCE_DIR}" rev-parse HEAD)
if [[ "${actual_commit}" != "${EXPECTED_COMMIT}" ]]; then
    echo "Expected Amiga commit ${EXPECTED_COMMIT}, found ${actual_commit}" >&2
    exit 1
fi

git -C "${SOURCE_DIR}" submodule update --init ros2-kortex-control
make -C "${SOURCE_DIR}" build-image \
    BASE_IMAGE=ghcr.io/ucmercedrobotics/ros2-kortex-control@sha256:5cb3739dff86174f6167823709d5e157300ee5160df858b2beef37f70dc1b1d3 \
    PLATFORM=linux/amd64 \
    IMAGE_TAG=amd64

"${COMPOSE[@]}" up -d ros
"${COMPOSE[@]}" exec -T ros bash -ic '
    set -e
    colcon build --packages-up-to amiga_ros2_gazebo kortex_move --cmake-args -DCMAKE_BUILD_TYPE=Release
    source /amiga-ros2-bridge/install/setup.bash
    ros2 pkg prefix amiga_ros2_gazebo
    ros2 pkg prefix kortex_move
'

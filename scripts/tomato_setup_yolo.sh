#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/tomato_common.sh"
tomato_enable_logging tomato_yolo_setup

if python3 -c 'import ensurepip' 2>/dev/null; then
    python3 -m venv "${TOMATO_YOLO_VENV}"
else
    python3 -m venv --without-pip "${TOMATO_YOLO_VENV}"
    curl -fsSL https://bootstrap.pypa.io/get-pip.py -o "${TOMATO_YOLO_VENV}/get-pip.py"
    "${TOMATO_YOLO_VENV}/bin/python" "${TOMATO_YOLO_VENV}/get-pip.py"
    rm -f "${TOMATO_YOLO_VENV}/get-pip.py"
fi
YOLO_DEVICE="$(tomato_yolo_device)"
case "${YOLO_DEVICE}" in
    cpu)
        PYTORCH_INDEX_URL=https://download.pytorch.org/whl/cpu
        TORCH_PACKAGE=torch==2.14.1+cpu
        TORCHVISION_PACKAGE=torchvision==0.29.1+cpu
        ;;
    0|cuda|cuda:0)
        PYTORCH_INDEX_URL=https://download.pytorch.org/whl/cu130
        TORCH_PACKAGE=torch==2.14.1+cu130
        TORCHVISION_PACKAGE=torchvision==0.29.1+cu130
        ;;
    *)
        printf 'TOMATO_DEVICE must be cpu or cuda:0, not %s\n' "${YOLO_DEVICE}" >&2
        exit 1
        ;;
esac

"${TOMATO_YOLO_VENV}/bin/python" -m pip install --upgrade pip setuptools wheel
"${TOMATO_YOLO_VENV}/bin/python" -m pip install \
    --index-url "${PYTORCH_INDEX_URL}" \
    "${TORCH_PACKAGE}" "${TORCHVISION_PACKAGE}"
"${TOMATO_YOLO_VENV}/bin/python" -m pip install \
    'numpy>=1.26,<2' \
    ultralytics \
    'opencv-python>=4.8,<4.12'

TOMATO_YOLO_MODEL="${TOMATO_YOLO_MODEL}" TOMATO_DEVICE="${YOLO_DEVICE}" \
    "${TOMATO_YOLO_VENV}/bin/python" - <<'PY'
import importlib.metadata
import os

import cv2
import torch
import ultralytics
from ultralytics import YOLO

device = os.environ["TOMATO_DEVICE"]
if device == "cpu":
    assert not torch.cuda.is_available(), "CPU-only setup unexpectedly reports CUDA"
    assert torch.version.cuda is None, f"Non-CPU PyTorch build: {torch.__version__}"
else:
    assert torch.cuda.is_available(), "CUDA requested but unavailable"
    assert torch.version.cuda is not None, f"CPU PyTorch build: {torch.__version__}"
    torch.zeros(1, device="cuda")
    print(f"gpu={torch.cuda.get_device_name(0)}")
model = YOLO(os.environ["TOMATO_YOLO_MODEL"])
print(f"torch={torch.__version__}")
print(f"torchvision={importlib.metadata.version('torchvision')}")
print(f"ultralytics={ultralytics.__version__}")
print(f"opencv={cv2.__version__}")
print(f"torch_cuda={torch.version.cuda}")
print(f"cuda_available={torch.cuda.is_available()}")
print(f"model_classes={model.names}")
PY

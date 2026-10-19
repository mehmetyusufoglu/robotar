#!/usr/bin/env bash
set -euo pipefail

if (( EUID != 0 )); then
    printf 'Run with sudo: sudo %s\n' "$0" >&2
    exit 1
fi

source /etc/os-release
if [[ "${ID}" != "ubuntu" || "${VERSION_CODENAME}" != "noble" ]]; then
    printf 'This installer requires Ubuntu 24.04 (noble); detected %s.\n' "${PRETTY_NAME}" >&2
    exit 1
fi

export DEBIAN_FRONTEND=noninteractive
KERNEL_RELEASE="$(uname -r)"

apt-get update
apt-get install -y \
    dkms \
    "linux-headers-${KERNEL_RELEASE}" \
    nvidia-driver-595-open

dkms autoinstall -k "${KERNEL_RELEASE}"
depmod -a "${KERNEL_RELEASE}"

printf '\nNVIDIA 595 open driver packages are installed for kernel %s.\n' "${KERNEL_RELEASE}"
dkms status -k "${KERNEL_RELEASE}"
if modprobe nvidia && modprobe nvidia-modeset && modprobe nvidia-uvm && nvidia-smi; then
    printf '\nThe NVIDIA driver is active; no additional reboot is required.\n'
else
    printf '\nThe driver was built but could not be activated in the current session.\n'
    printf 'Reboot and then verify it with: nvidia-smi\n'
fi

#!/usr/bin/env bash

IMAGE_NAME="kernel-2635-kernel-builder"
CONTAINER_NAME="kernel-build-env"

# Ensure we run from the script's root directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TOOLCHAIN_HOST_DIR="/home/hillbeast/Development/cross/codesourcery-2011.03"
TOOLCHAIN_CONTAINER_DIR="/opt/toolchains/codesourcery-2011.03"

echo "Entering build container (${IMAGE_NAME})..."

docker run --rm -it \
  --name "${CONTAINER_NAME}" \
  -v "${SCRIPT_DIR}:/workspace" \
  -v "${TOOLCHAIN_HOST_DIR}:${TOOLCHAIN_CONTAINER_DIR}:ro" \
  -w /workspace \
  -u "$(id -u):$(id -g)" \
  -e ARCH=arm \
  -e CROSS_COMPILE=arm-none-eabi- \
  -e PATH="${TOOLCHAIN_CONTAINER_DIR}/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  "${IMAGE_NAME}" bash
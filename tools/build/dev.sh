#!/usr/bin/env bash
# Build the stable dependency image and the host-mounted ROS contract workspace.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

DOCKER="${DEV_DOCKER:-docker}"
IMAGE="${DEV_IMAGE:-sim-dual-arm/dev-ros:stage2}"
BASE_IMAGE="${DEV_ROS_BASE_IMAGE:-sim-dual-arm/ros-base:stage2}"
BUILD_ROOT="${DEV_BUILD_ROOT:-$ROOT/.build/ros-dev}"

export DEV_IMAGE="$IMAGE"
export DEV_BUILD_ROOT="$BUILD_ROOT"

build_image() {
  "$DOCKER" build \
    --build-arg "ROS_BASE_IMAGE=$BASE_IMAGE" \
    -t "$IMAGE" \
    -f deploy/docker/dev/Dockerfile \
    "$ROOT"
}

build_contracts() {
  python3 "$ROOT/tools/run/dev_compose.py" \
    --image "$IMAGE" \
    --build-root "$BUILD_ROOT" \
    build-contracts
}

case "${1:-all}" in
  all)
    build_image
    build_contracts
    ;;
  image)
    build_image
    ;;
  contracts)
    build_contracts
    ;;
  *)
    echo "usage: bash tools/build/dev.sh [all|image|contracts]" >&2
    exit 2
    ;;
esac

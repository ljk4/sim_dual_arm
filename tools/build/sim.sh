#!/usr/bin/env bash
# Build only the stable simulation dependency target.
# Source, contracts, assets and configuration stay on read-only runtime mounts.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

IMAGE="${SIM_IMAGE:-sim-dual-arm/backend-sim:mujoco-dev}"
SIM_BASE_IMAGE="${SIM_BASE_IMAGE:-sim-dual-arm/dev-ros:stage2}"
DOCKER="${DEV_DOCKER:-docker}"

"$DOCKER" build \
  --build-arg "SIM_BASE_IMAGE=$SIM_BASE_IMAGE" \
  --target mujoco-dev \
  -t "$IMAGE" \
  -f deploy/docker/backend-sim/Dockerfile \
  "$ROOT"

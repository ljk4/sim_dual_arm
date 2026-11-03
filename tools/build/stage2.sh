#!/usr/bin/env bash
# Build dependencies and generated SDKs for the source-mounted development stack.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DOCKER="${DEV_DOCKER:-docker}"
ROS_BASE_IMAGE="${DEV_ROS_BASE_IMAGE:-sim-dual-arm/ros-base:stage2}"
PROTOCOL_IMAGE="${PROTOCOL_IMAGE:-sim-dual-arm/learning:protocol}"
POLICY_IMAGE="${POLICY_IMAGE:-sim-dual-arm/learning:policy-dev}"
cd "$ROOT"

"$DOCKER" build -t "$ROS_BASE_IMAGE" -f deploy/docker/ros-base/Dockerfile .
export DEV_ROS_BASE_IMAGE="$ROS_BASE_IMAGE"
bash tools/build/dev.sh
bash tools/build/policy_sdk.sh
"$DOCKER" build --target protocol -t "$PROTOCOL_IMAGE" -f deploy/docker/learning/Dockerfile .
"$DOCKER" build --target policy -t "$POLICY_IMAGE" -f deploy/docker/learning/Dockerfile .

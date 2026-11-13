#!/usr/bin/env bash
# Build the ROS contract workspace with the container's Python and colcon.
set -euo pipefail
SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
SHARED_ROOT="${SHARED_ROOT:-$SCRIPT_ROOT/shared}"
source "$SHARED_ROOT/common/tools/docker/setup.sh"

ROOT="${PROJECT_ROOT:-$SCRIPT_ROOT}"
BUILD_ROOT="${BUILD_ROOT:-/build}"
# One build covers every common and scenario-extension interface package.
SDK_BUILD_ROOT="${ROS_SDK_BUILD_ROOT:-$BUILD_ROOT/ros-dev}"
python3 "$SHARED_ROOT/common/tools/scripts/dev_workspace.py" build \
  --project-root "$ROOT" \
  --build-root "$SDK_BUILD_ROOT" \
  "$@"

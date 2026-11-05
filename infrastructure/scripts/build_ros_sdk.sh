#!/usr/bin/env bash
# Build the ROS contract workspace with the container's Python and colcon.
set -euo pipefail
SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$SCRIPT_ROOT/infrastructure/docker/setup.sh"

ROOT="${PROJECT_ROOT:-$SCRIPT_ROOT}"
BUILD_ROOT="${BUILD_ROOT:-/build}"
SDK_BUILD_ROOT="${ROS_SDK_BUILD_ROOT:-$BUILD_ROOT/ros-dev}"
python3 "$ROOT/infrastructure/scripts/dev_workspace.py" build \
  --project-root "$ROOT" \
  --build-root "$SDK_BUILD_ROOT" \
  "$@"

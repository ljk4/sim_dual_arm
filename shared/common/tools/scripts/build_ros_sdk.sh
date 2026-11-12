#!/usr/bin/env bash
# Build the ROS contract workspace with the container's Python and colcon.
set -euo pipefail
SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
SHARED_ROOT="${SHARED_ROOT:-$SCRIPT_ROOT/shared}"
source "$SHARED_ROOT/common/tools/docker/setup.sh"

ROOT="${PROJECT_ROOT:-$SCRIPT_ROOT}"
BUILD_ROOT="${BUILD_ROOT:-/build}"
# A scenario/package selection builds the full SDK at the active ros-dev root.
# A no-selection invocation builds only the core package into a separate root so
# it can never replace an active full scenario SDK.
selection=false
for argument in "$@"; do
  case "$argument" in
    --scenario|--scenario=*|--package|--package=*) selection=true ;;
  esac
done
if [[ "$selection" == "true" ]]; then
  SDK_BUILD_ROOT="${ROS_SDK_BUILD_ROOT:-$BUILD_ROOT/ros-dev}"
else
  SDK_BUILD_ROOT="${ROS_CORE_SDK_BUILD_ROOT:-$BUILD_ROOT/ros-core}"
fi
python3 "$SHARED_ROOT/common/tools/scripts/dev_workspace.py" build \
  --project-root "$ROOT" \
  --build-root "$SDK_BUILD_ROOT" \
  "$@"

#!/usr/bin/env bash
# Run pytest inside the runtime container with ROS SDK verified.
set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-${SHARED_ROOT:-$APP_ROOT/../../shared}/common/tools/docker/setup.sh}"

ROOT="${PROJECT_ROOT:-$APP_ROOT}"
ROS_SDK="${ROS_SDK_ROOT:-/build/ros-dev}"

has_help=false
for argument in "$@"; do
  if [[ "$argument" == "-h" || "$argument" == "--help" ]]; then
    has_help=true
  fi
done

if [[ "$has_help" != "true" ]]; then
  python3 "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/dev_workspace.py" check \
    --project-root "$ROOT" \
    --build-root "$ROS_SDK" 1>&2
fi

exec bash "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/test.sh" "$@"

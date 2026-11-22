#!/usr/bin/env bash
# Run one non-interactive policy execution inside the runtime container.
set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-${SHARED_ROOT:-$APP_ROOT/../../shared}/common/tools/docker/setup.sh}"

ROOT="${PROJECT_ROOT:-$APP_ROOT}"
ROS_SDK="${ROS_SDK_ROOT:-/build/ros-dev}"
POLICY_SDK="${POLICY_SDK_ROOT:-/build/generated/policy-sdk}"

if [[ " $* " != *" -h "* && " $* " != *" --help "* ]]; then
  python3 "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/dev_workspace.py" check \
    --project-root "$ROOT" \
    --build-root "$ROS_SDK" 1>/dev/null
  python3 "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/policy_sdk.py" check \
    --project-root "$ROOT" \
    --sdk-root "$POLICY_SDK" >/dev/null 2>&1
fi

exec python3 -m dual_arm_runtime.runtime_node \
  --project-root "$ROOT" \
  --once \
  "$@"

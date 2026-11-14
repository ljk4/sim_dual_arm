#!/usr/bin/env bash
# Run headless batch policy inference against a simulation backend inside the runtime container.
set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-${SHARED_ROOT:-$APP_ROOT/../../shared}/common/tools/docker/setup.sh}"

ROOT="${PROJECT_ROOT:-$APP_ROOT}"
ROS_SDK="${ROS_SDK_ROOT:-/build/ros-dev}"
POLICY_SDK="${POLICY_SDK_ROOT:-/build/generated/policy-sdk}"

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
  python3 "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/policy_sdk.py" check \
    --project-root "$ROOT" \
    --sdk-root "$POLICY_SDK" 1>&2 >/dev/null
fi

exec python3 -m dual_arm_runtime.batch_policy --project-root "$ROOT" "$@"

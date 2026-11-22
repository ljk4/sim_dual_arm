#!/usr/bin/env bash
# Serve the named ACT Policy API (ROS-free) inside the learning container.
set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-${SHARED_ROOT:-$APP_ROOT/../../shared}/common/tools/docker/setup.sh}"

ROOT="${PROJECT_ROOT:-$APP_ROOT}"
POLICY_SDK="${POLICY_SDK_ROOT:-/opt/policy-sdk}"

if [[ " $* " != *" -h "* && " $* " != *" --help "* ]]; then
  python3 "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/policy_sdk.py" check \
    --project-root "$ROOT" \
    --sdk-root "$POLICY_SDK" >/dev/null
fi

exec python3 -m dual_arm_learning.serving.cli "$@"

#!/usr/bin/env bash
# Serve the named ACT Policy API (ROS-free) inside the learning container.
set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-${SHARED_ROOT:-$APP_ROOT/../../shared}/common/tools/docker/setup.sh}"

ROOT="${PROJECT_ROOT:-$APP_ROOT}"
DATA_ROOT="${DATA_ROOT:-/data}"
POLICY_SDK="${POLICY_SDK_ROOT:-/build/generated/policy-sdk}"
ACT_LOG_DIRECTORY="${ACT_LOG_DIRECTORY:-}"

has_help=false
scenario="_direct"
previous=""
for argument in "$@"; do
  if [[ "$argument" == "-h" || "$argument" == "--help" ]]; then
    has_help=true
  fi
  if [[ "$previous" == "--scenario" ]]; then
    scenario="$argument"
  elif [[ "$argument" == --scenario=* ]]; then
    scenario="${argument#*=}"
  fi
  previous="$argument"
done

if [[ "$has_help" != "true" ]]; then
  python3 "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/policy_sdk.py" check \
    --project-root "$ROOT" \
    --sdk-root "$POLICY_SDK" 1>&2 >/dev/null
fi

log_directory="$ACT_LOG_DIRECTORY"
if [[ -z "$log_directory" ]]; then
  log_directory="$DATA_ROOT/$scenario/logs/policy-service"
fi

exec python3 -m dual_arm_learning.policy_service_cli \
  --log-directory "$log_directory" \
  --run-id "${ACT_RUN_ID:-policy-$(date -u +%Y%m%dT%H%M%SZ)-$$}" \
  "$@"

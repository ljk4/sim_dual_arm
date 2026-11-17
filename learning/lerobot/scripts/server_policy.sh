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
robot=""
task=""
previous=""
for argument in "$@"; do
  if [[ "$argument" == "-h" || "$argument" == "--help" ]]; then
    has_help=true
  fi
  if [[ "$previous" == "--robot" ]]; then
    robot="$argument"
  elif [[ "$argument" == --robot=* ]]; then
    robot="${argument#*=}"
  elif [[ "$previous" == "--task" ]]; then
    task="$argument"
  elif [[ "$argument" == --task=* ]]; then
    task="${argument#*=}"
  fi
  previous="$argument"
done

if [[ "$has_help" != "true" ]]; then
  python3 "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/policy_sdk.py" check \
    --project-root "$ROOT" \
    --sdk-root "$POLICY_SDK" 1>&2 >/dev/null
fi

run_id="${ACT_RUN_ID:-policy-$(date -u +%Y%m%dT%H%M%SZ)-$$}"
log_directory="$ACT_LOG_DIRECTORY"
if [[ -z "$log_directory" ]]; then
  if [[ -n "$robot" && -n "$task" ]]; then
    log_directory="$DATA_ROOT/$robot/$task/logs/policy-service/$run_id"
  else
    log_directory="$DATA_ROOT/_direct/logs/policy-service/$run_id"
  fi
fi

exec python3 -m dual_arm_learning.policy_service_cli \
  --log-directory "$log_directory" \
  --run-id "$run_id" \
  "$@"

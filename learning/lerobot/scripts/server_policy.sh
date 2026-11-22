#!/usr/bin/env bash
# Serve the named ACT Policy API (ROS-free) inside the learning container.
set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-${SHARED_ROOT:-$APP_ROOT/../../shared}/common/tools/docker/setup.sh}"

ROOT="${PROJECT_ROOT:-$APP_ROOT}"
DATA_ROOT="${DATA_ROOT:-/data}"
POLICY_SDK="${POLICY_SDK_ROOT:-/opt/policy-sdk}"
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
    --sdk-root "$POLICY_SDK" >/dev/null
fi

run_id="${ACT_RUN_ID:-policy_$(date +%Y%m%d_%H%M%S)}"
log_directory="$ACT_LOG_DIRECTORY"
if [[ -z "$log_directory" ]]; then
  if [[ -n "$robot" && -n "$task" ]]; then
    service_root="$DATA_ROOT/$robot/$task/logs/policy-service"
  else
    service_root="$DATA_ROOT/_direct/logs/policy-service"
  fi
  if [[ -z "${ACT_RUN_ID:-}" && -e "$service_root/$run_id" ]]; then
    suffix=2
    while [[ -e "$service_root/${run_id}_${suffix}" ]]; do
      suffix=$((suffix + 1))
    done
    run_id="${run_id}_${suffix}"
  fi
  log_directory="$service_root/$run_id"
fi

exec python3 -m dual_arm_learning.serving.cli \
  --log-directory "$log_directory" \
  --run-id "$run_id" \
  "$@"

#!/usr/bin/env bash
# Run one non-interactive policy execution inside the runtime container.
set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-${SHARED_ROOT:-$APP_ROOT/../../shared}/common/tools/docker/setup.sh}"

ROOT="${PROJECT_ROOT:-$APP_ROOT}"
DATA_ROOT="${DATA_ROOT:-/data}"
ROS_SDK="${ROS_SDK_ROOT:-/build/ros-dev}"
POLICY_SDK="${POLICY_SDK_ROOT:-/build/generated/policy-sdk}"

option_value() {
  local name="$1"
  shift
  local previous="" argument
  for argument in "$@"; do
    if [[ "$previous" == "$name" ]]; then
      printf '%s\n' "$argument"
      return 0
    fi
    case "$argument" in
      "$name"=*) printf '%s\n' "${argument#*=}"; return 0 ;;
    esac
    previous="$argument"
  done
  return 1
}

has_help=false
for argument in "$@"; do
  if [[ "$argument" == "-h" || "$argument" == "--help" ]]; then
    has_help=true
  fi
done

robot="$(option_value --robot "$@" || true)"
task="$(option_value --task "$@" || true)"
if [[ -n "$robot" && -n "$task" ]]; then
  segment="$robot/$task"
else
  segment="_direct"
fi

if [[ "$has_help" != "true" ]]; then
  python3 "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/dev_workspace.py" check \
    --project-root "$ROOT" \
    --build-root "$ROS_SDK" 1>/dev/null
  python3 "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/policy_sdk.py" check \
    --project-root "$ROOT" \
    --sdk-root "$POLICY_SDK" >/dev/null 2>&1
fi

run_id="$(option_value --run-id "$@" || true)"
if [[ -z "$run_id" ]]; then
  run_id="policy_$(date +%Y%m%d_%H%M%S)"
  rec_dir="$DATA_ROOT/$segment/logs/recordings"
  if [[ -e "$rec_dir/$run_id" ]]; then
    suffix=2
    while [[ -e "$rec_dir/${run_id}_${suffix}" ]]; do
      suffix=$((suffix + 1))
    done
    run_id="${run_id}_${suffix}"
  fi
fi
defaults=()
option_value --run-id "$@" >/dev/null || defaults+=(--run-id "$run_id")
option_value --episode-id "$@" >/dev/null || defaults+=(--episode-id "$run_id")
option_value --output "$@" >/dev/null || \
  defaults+=(--output "$DATA_ROOT/$segment/logs/recordings")

exec python3 -m dual_arm_runtime.runtime_node \
  --project-root "$ROOT" \
  --runs-output "$DATA_ROOT/$segment/logs/runs" \
  --once \
  "${defaults[@]}" \
  "$@"

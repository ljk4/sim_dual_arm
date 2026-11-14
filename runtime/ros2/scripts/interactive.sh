#!/usr/bin/env bash
# Prompt, reset via episode start, run one policy episode, and hold until Ctrl-C.
set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-${SHARED_ROOT:-$APP_ROOT/../../shared}/common/tools/docker/setup.sh}"

ROOT="${PROJECT_ROOT:-$APP_ROOT}"
DATA_ROOT="${DATA_ROOT:-/data}"
ROS_SDK="${ROS_SDK_ROOT:-/build/ros-dev}"
POLICY_SDK="${POLICY_SDK_ROOT:-/build/generated/policy-sdk}"

config=""
scenario=""
role="policy"
shared_root=""
bundle=""
run_id=""
seed=""
output_root=""
selection_args=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      config="${2:-}"
      shift 2
      ;;
    --config=*)
      config="${1#*=}"
      shift
      ;;
    --scenario)
      scenario="${2:-}"
      shift 2
      ;;
    --scenario=*)
      scenario="${1#*=}"
      shift
      ;;
    --role)
      role="${2:-}"
      shift 2
      ;;
    --role=*)
      role="${1#*=}"
      shift
      ;;
    --shared-root)
      shared_root="${2:-}"
      shift 2
      ;;
    --shared-root=*)
      shared_root="${1#*=}"
      shift
      ;;
    --bundle)
      bundle="${2:-}"
      shift 2
      ;;
    --bundle=*)
      bundle="${1#*=}"
      shift
      ;;
    --set)
      selection_args+=(--set "${2:-}")
      shift 2
      ;;
    --set=*)
      selection_args+=(--set "${1#*=}")
      shift
      ;;
    --run-id)
      run_id="${2:-}"
      shift 2
      ;;
    --run-id=*)
      run_id="${1#*=}"
      shift
      ;;
    --seed)
      seed="${2:-}"
      shift 2
      ;;
    --seed=*)
      seed="${1#*=}"
      shift
      ;;
    --output-root)
      output_root="${2:-}"
      shift 2
      ;;
    --output-root=*)
      output_root="${1#*=}"
      shift
      ;;
    -h|--help)
      printf '%s\n' \
        'usage: scripts/interactive.sh (--scenario <id> [--role policy|teacher] [--shared-root <dir>] [--bundle <dir>] | --config <runtime.yaml>) [--run-id <id>] [--seed <int>] [--output-root <dir>] [--set path=value ...]'
      exit 0
      ;;
    *)
      printf 'interactive: unknown argument %q\n' "$1" >&2
      exit 2
      ;;
  esac
done

if [[ -n "$config" && -n "$scenario" ]]; then
  printf 'interactive: --config and --scenario are mutually exclusive\n' >&2
  exit 2
fi
if [[ -z "$config" && -z "$scenario" ]]; then
  printf 'interactive: one of --scenario or --config is required\n' >&2
  exit 2
fi
if [[ -n "$config" ]]; then
  selection_args=(--config "$config" "${selection_args[@]}")
  display="$config"
  segment="_direct"
else
  selection_args=(--scenario "$scenario" --role "$role" "${selection_args[@]}")
  if [[ -n "$shared_root" ]]; then
    selection_args+=(--shared-root "$shared_root")
  fi
  if [[ -n "$bundle" ]]; then
    selection_args+=(--bundle "$bundle")
  fi
  display="scenario:$scenario/$role"
  segment="$scenario"
fi
if [[ -z "$output_root" ]]; then
  output_root="$DATA_ROOT/$segment/logs/recordings"
fi
if [[ -z "$run_id" ]]; then
  run_id="interactive-$(date -u +%Y%m%dT%H%M%SZ)-$$"
fi
if [[ -n "$seed" && ! "$seed" =~ ^[0-9]+$ ]]; then
  printf 'interactive: --seed must be a non-negative integer\n' >&2
  exit 2
fi
if [[ -e "$output_root/$run_id" ]]; then
  printf 'interactive: output already exists: %s\n' "$output_root/$run_id" >&2
  exit 2
fi
printf '配置: %s\n' "$display"
printf 'run_id: %s\n' "$run_id"
printf 'seed: %s\n' "${seed:-<runtime reset.seed>}"
printf '输入 y 复原环境并开始本次任务，其他键取消: '
if ! read -r answer; then
  printf '\ninteractive: no answer; canceled\n' >&2
  exit 2
fi
case "$answer" in
  y|Y|yes|YES)
    ;;
  *)
    printf 'interactive: canceled\n'
    exit 0
    ;;
esac

python3 "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/dev_workspace.py" check \
  --project-root "$ROOT" \
  --build-root "$ROS_SDK" 1>&2
python3 "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/policy_sdk.py" check \
  --project-root "$ROOT" \
  --sdk-root "$POLICY_SDK" 1>&2 >/dev/null

seed_args=()
if [[ -n "$seed" ]]; then
  seed_args=(--reset-seed "$seed")
fi
exec python3 -m dual_arm_runtime.runtime_node \
  --project-root "$ROOT" \
  "${selection_args[@]}" \
  --runs-output "$DATA_ROOT/$segment/logs/runs" \
  --once \
  --hold-after-episode \
  --run-id "$run_id" \
  --episode-id "$run_id" \
  --output "$output_root" \
  "${seed_args[@]}"

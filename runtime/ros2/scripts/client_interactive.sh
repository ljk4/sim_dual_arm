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
robot=""
task=""
role="policy"
policy_endpoint=""
shared_root=""
run_id=""
seed=""
output_root=""

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
    --robot)
      robot="${2:-}"
      shift 2
      ;;
    --robot=*)
      robot="${1#*=}"
      shift
      ;;
    --task)
      task="${2:-}"
      shift 2
      ;;
    --task=*)
      task="${1#*=}"
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
    --policy-endpoint)
      policy_endpoint="${2:?missing value for --policy-endpoint}"
      shift 2
      ;;
    --policy-endpoint=*)
      policy_endpoint="${1#*=}"
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
        'usage: scripts/client_interactive.sh (--robot <id> --task <id> [--role policy|teacher] [--shared-root <dir>] | --config <runtime.yaml>) [--run-id <id>] [--seed <int>] [--output-root <dir>]' \
        '  --policy-endpoint HOST:PORT  Policy Service gRPC address.'
      exit 0
      ;;
    *)
      printf 'interactive: unknown argument %q\n' "$1" >&2
      exit 2
      ;;
  esac
done

scenario=false
if [[ -n "$robot" || -n "$task" ]]; then
  scenario=true
fi
if [[ -n "$config" && "$scenario" == "true" ]]; then
  printf 'interactive: --config and --robot/--task are mutually exclusive\n' >&2
  exit 2
fi
if [[ -n "$config" ]]; then
  segment="_direct"
elif [[ -n "$robot" && -n "$task" ]]; then
  segment="$robot/$task"
else
  printf 'interactive: one of --robot/--task or --config is required\n' >&2
  exit 2
fi

if [[ -n "$config" ]]; then
  selection_args=(--config "$config")
  display="$config"
else
  selection_args=(--robot "$robot" --task "$task" --role "$role")
  if [[ -n "$shared_root" ]]; then
    selection_args+=(--shared-root "$shared_root")
  fi
  display="scenario:$robot/$task/$role"
fi
if [[ -n "$policy_endpoint" ]]; then
  selection_args+=(--policy-endpoint "$policy_endpoint")
fi
if [[ -z "$output_root" ]]; then
  output_root="$DATA_ROOT/$segment/logs/recordings"
fi
ts="$(date +%Y%m%d_%H%M%S)"
base="${run_id:-interactive}"
if [[ ! "$base" =~ (_[0-9]{8}_[0-9]{6}|-[0-9]{8}T[0-9]{6}Z)$ ]]; then
  run_id="${base}_${ts}"
else
  run_id="$base"
fi
if [[ -e "$output_root/$run_id" ]]; then
  suffix=2
  while [[ -e "$output_root/${run_id}_${suffix}" ]]; do
    suffix=$((suffix + 1))
  done
  run_id="${run_id}_${suffix}"
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
if [[ -n "$policy_endpoint" ]]; then
  printf '策略服务: %s\n' "$policy_endpoint"
fi
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
  --build-root "$ROS_SDK" 1>/dev/null
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

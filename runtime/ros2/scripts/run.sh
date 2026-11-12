#!/usr/bin/env bash
# Runtime command dispatcher for manual use inside the idle container.
# It selects the real CLI or SDK builder and forwards arguments unchanged.
set -euo pipefail

SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "${PROJECT_SETUP:-${SHARED_ROOT:-$SCRIPT_ROOT/shared}/common/tools/docker/setup.sh}"

ROOT="${PROJECT_ROOT:-$SCRIPT_ROOT}"
DATA_ROOT="${DATA_ROOT:-/data}"
ROS_SDK="${ROS_SDK_ROOT:-/build/ros-dev}"
POLICY_SDK="${POLICY_SDK_ROOT:-/build/generated/policy-sdk}"
DEFAULT_POLICY_CONFIG="${SHARED_ROOT:-/shared}/scenarios/panda_stack/configs/runtime/policy_rollout.yaml"

has_help=false
for argument in "$@"; do
  if [[ "$argument" == "-h" || "$argument" == "--help" ]]; then
    has_help=true
  fi
done

require_output() {
  if [[ "$has_help" == "true" ]]; then
    return 0
  fi
  local command_name="$1"
  shift
  for argument in "$@"; do
    if [[ "$argument" == "--output" || "$argument" == --output=* ]]; then
      return 0
    fi
  done
  printf '%s: --output /data/raw/<run-id> is required for each run\n' "$command_name" >&2
  return 2
}

check_ros_sdk() {
  if [[ "$has_help" == "true" ]]; then
    return 0
  fi
  python3 "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/dev_workspace.py" check \
    --project-root "$ROOT" \
    --build-root "$ROS_SDK" 1>&2
}

check_policy_sdk() {
  if [[ "$has_help" == "true" ]]; then
    return 0
  fi
  python3 "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/policy_sdk.py" check \
    --project-root "$ROOT" \
    --sdk-root "$POLICY_SDK" 1>&2 >/dev/null
}

usage() {
  cat <<'EOF'
usage: runtime/ros2/scripts/run.sh <command> [arguments...]

commands:
  build-ros-sdk   build the mounted ROS contract workspace (core-only unless --scenario)
  build-policy-sdk generate the mounted Policy SDK in the container
  prepare         bind a scenario/direct recipe to runtime.yaml + run_manifest.json
  policy          run one policy execution (requires --output)
  interactive     prompt, reset via episode start, run one policy episode, hold until Ctrl-C
  export          export one episode to labels/RGB/quality (export_episode.py)
  campaign        inspect budgets, select data, or run a custom attempt (campaign.py)
  evaluate        build a read-only evaluation report (evaluate.py)
  test            run pytest with plugin autoload disabled

Examples:
  runtime/ros2/scripts/run.sh build-ros-sdk --scenario panda_stack --shared-root /shared
  runtime/ros2/scripts/run.sh build-policy-sdk
  runtime/ros2/scripts/run.sh prepare --help
  runtime/ros2/scripts/run.sh prepare --scenario panda_stack --role policy \
    --policy-binding /data/prepared/policy/policy_binding.json \
    --bundle /data/learning/bundles/<verified-bundle> \
    --seed 20001 --run-id policy-1 --output /data/prepared/policy-1
  runtime/ros2/scripts/run.sh prepare --scenario panda_stack --role teacher \
    --seed 1000 --run-id teacher-1 --output /data/prepared/teacher-1
  runtime/ros2/scripts/run.sh policy \
    --config /data/prepared/policy-1/runtime.yaml --output /data/raw/policy-1
  runtime/ros2/scripts/record.sh --count 10 --max-consecutive-failures 3
  runtime/ros2/scripts/run.sh export --episode-dir /data/raw/e-001 \
    --out /data/exports/e-001
  runtime/ros2/scripts/run.sh test -q tests/unit
EOF
}

command="${1:-}"
if [[ $# -gt 0 ]]; then
  shift
fi
case "$command" in
  ""|-h|--help|help)
    usage
    ;;
  build-ros-sdk)
    exec bash "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/build_ros_sdk.sh" "$@"
    ;;
  build-policy-sdk)
    exec bash "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/build_policy_sdk.sh" "$@"
    ;;
  prepare)
    exec python3 -m dual_arm_runtime.offline.prepare_run "$@"
    ;;
  policy)
    require_output "policy" "$@" || exit 2
    check_ros_sdk
    check_policy_sdk
    exec python3 -m dual_arm_runtime.runtime_node \
      --project-root "$ROOT" \
      --config "${RUNTIME_CONFIG:-$DEFAULT_POLICY_CONFIG}" \
      --runs-output "$DATA_ROOT/runs" \
      --once \
      "$@"
    ;;
  interactive)
    config=""
    run_id=""
    seed=""
    output_root="$DATA_ROOT/raw"
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
            'usage: runtime/ros2/scripts/run.sh interactive --config <runtime.yaml> [--run-id <id>] [--seed <int>] [--output-root <dir>]'
          exit 0
          ;;
        *)
          printf 'interactive: unknown argument %q\n' "$1" >&2
          exit 2
          ;;
      esac
    done
    if [[ -z "$config" ]]; then
      printf 'interactive: --config <runtime.yaml> is required\n' >&2
      exit 2
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
    printf '配置: %s\n' "$config"
    printf 'run_id: %s\n' "$run_id"
    printf 'seed: %s\n' "${seed:-<runtime.yaml reset.seed>}"
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
    check_ros_sdk
    check_policy_sdk
    seed_args=()
    if [[ -n "$seed" ]]; then
      seed_args=(--reset-seed "$seed")
    fi
    exec python3 -m dual_arm_runtime.runtime_node \
      --project-root "$ROOT" \
      --config "$config" \
      --runs-output "$DATA_ROOT/runs" \
      --once \
      --hold-after-episode \
      --run-id "$run_id" \
      --episode-id "$run_id" \
      --output "$output_root/$run_id" \
      "${seed_args[@]}"
    ;;
  export)
    check_ros_sdk
    exec python3 -m dual_arm_runtime.offline.export_episode "$@"
    ;;
  campaign)
    check_ros_sdk
    exec python3 -m dual_arm_runtime.offline.campaign \
      --project-root "$ROOT" "$@"
    ;;
  evaluate)
    check_ros_sdk
    exec python3 -m dual_arm_runtime.offline.evaluate \
      --project-root "$ROOT" "$@"
    ;;
  test)
    check_ros_sdk
    exec bash "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/test.sh" "$@"
    ;;
  *)
    printf 'runtime run.sh: unknown command %q\n' "$command" >&2
    usage >&2
    exit 2
    ;;
esac

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

has_help=false
for argument in "$@"; do
  if [[ "$argument" == "-h" || "$argument" == "--help" ]]; then
    has_help=true
  fi
done

scenario_segment() {
  local previous="" argument
  for argument in "$@"; do
    if [[ "$previous" == "--scenario" ]]; then
      printf '%s\n' "$argument"
      return 0
    fi
    case "$argument" in
      --scenario=*) printf '%s\n' "${argument#*=}"; return 0 ;;
    esac
    previous="$argument"
  done
  printf '_direct\n'
}

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
  build-ros-sdk   build all ROS interface packages into the mounted SDK workspace
  build-policy-sdk generate the mounted Policy SDK in the container
  policy          run one policy execution
  interactive     prompt, reset via episode start, run one policy episode, hold until Ctrl-C
  export          export one episode to labels/RGB/quality (export_episode.py)
  campaign        inspect budgets, select data, or run a custom attempt (campaign.py)
  evaluate        build a read-only evaluation report (evaluate.py)
  test            run pytest with plugin autoload disabled

Examples:
  runtime/ros2/scripts/run.sh build-ros-sdk --shared-root /shared
  runtime/ros2/scripts/run.sh build-policy-sdk
  runtime/ros2/scripts/run.sh policy --scenario panda_stack \
    --bundle /data/panda_stack/models/bundles/stack-act \
    --reset-seed 20001 --run-id policy-1
  runtime/ros2/scripts/run.sh interactive --scenario panda_stack \
    --bundle /data/panda_stack/models/bundles/stack-act --seed 20002
  runtime/ros2/scripts/run.sh policy --config /shared/scenarios/panda_stack/configs/runtime/policy_rollout.yaml \
    --run-id policy-1
  runtime/ros2/scripts/record.sh --scenario panda_stack --count 10 --max-consecutive-failures 3
  runtime/ros2/scripts/run.sh export --scenario panda_stack \
    --episode-dir /data/panda_stack/logs/recordings/e-001 \
    --out /data/panda_stack/diagnostics/export/e-001
  runtime/ros2/scripts/run.sh test -q tests/unit

`policy` and `interactive` require exactly one of --scenario or --config; there is
no implicit scenario fallback.  Run logs and recordings default to
/data/<scenario-id>/logs/{runs,recordings} (`_direct` without a scenario).
--set dot.path=value overrides existing keys.
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
  policy)
    check_ros_sdk
    check_policy_sdk
    segment="$(scenario_segment "$@")"
    run_id="$(option_value --run-id "$@" || true)"
    if [[ -z "$run_id" ]]; then
      run_id="policy-$(date -u +%Y%m%dT%H%M%SZ)-$$"
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
    ;;
  interactive)
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
            'usage: runtime/ros2/scripts/run.sh interactive (--scenario <id> [--role policy|teacher] [--shared-root <dir>] [--bundle <dir>] | --config <runtime.yaml>) [--run-id <id>] [--seed <int>] [--output-root <dir>] [--set path=value ...]'
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
    check_ros_sdk
    check_policy_sdk
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

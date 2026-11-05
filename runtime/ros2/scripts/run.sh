#!/usr/bin/env bash
# Runtime command dispatcher for manual use inside the idle container.
# It selects the real CLI or SDK builder and forwards arguments unchanged.
set -euo pipefail

SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$SCRIPT_ROOT/infrastructure/docker/setup.sh"

ROOT="${PROJECT_ROOT:-$SCRIPT_ROOT}"
DATA_ROOT="${DATA_ROOT:-/data}"
ROS_SDK="${ROS_SDK_ROOT:-/opt/ros-dev}"
POLICY_SDK="${POLICY_SDK_ROOT:-/opt/policy-sdk}"
DEFAULT_RUNTIME_CONFIG="$ROOT/runtime/ros2/configs/native_collect.yaml"
DEFAULT_POLICY_CONFIG="$ROOT/runtime/ros2/configs/native_policy.yaml"

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
  python3 "$ROOT/infrastructure/scripts/dev_workspace.py" check \
    --project-root "$ROOT" \
    --build-root "$ROS_SDK" 1>&2
}

check_policy_sdk() {
  if [[ "$has_help" == "true" ]]; then
    return 0
  fi
  python3 "$ROOT/infrastructure/scripts/policy_sdk.py" check \
    --project-root "$ROOT" \
    --sdk-root "$POLICY_SDK" 1>&2 >/dev/null
}

usage() {
  cat <<'EOF'
usage: runtime/ros2/scripts/run.sh <command> [arguments...]

commands:
  build-ros-sdk   build the mounted ROS contract workspace in the container
  build-policy-sdk generate the mounted Policy SDK in the container
  prepare         bind a generic Policy JSON to runtime.yaml + run_manifest.json
  collect         run one native ROS collection (requires --output)
  policy          run one native policy execution (requires --output)
  control         use the public runtime status/start/end client
  audit           inspect one committed episode (audit.main dispatches native v3)
  export          export one episode to labels/RGB/quality (stack_export.py)
  campaign        reserve and run one budgeted campaign attempt (stack_campaign.py)
  evaluate        build a read-only evaluation report (stack_evaluate.py)
  test            run pytest with plugin autoload disabled

Examples:
  runtime/ros2/scripts/run.sh build-ros-sdk
  runtime/ros2/scripts/run.sh build-policy-sdk
  runtime/ros2/scripts/run.sh prepare --help
  runtime/ros2/scripts/run.sh collect --config \
    /workspace/runtime/ros2/configs/native_collect.yaml \
    --run-id collect-001 --output /data/raw/collect-001
  runtime/ros2/scripts/run.sh control --run-id dev-001 status
  runtime/ros2/scripts/run.sh audit /data/raw/collect-001
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
    exec bash "$ROOT/infrastructure/scripts/build_ros_sdk.sh" "$@"
    ;;
  build-policy-sdk)
    exec bash "$ROOT/infrastructure/scripts/build_policy_sdk.sh" "$@"
    ;;
  prepare)
    exec python3 "$ROOT/runtime/ros2/scripts/prepare_run.py" "$@"
    ;;
  collect)
    require_output "collect" "$@" || exit 2
    check_ros_sdk
    exec python3 -m dual_arm_runtime.native_node \
      --project-root "$ROOT" \
      --config "${NATIVE_RUNTIME_CONFIG:-$DEFAULT_RUNTIME_CONFIG}" \
      --runs-output "$DATA_ROOT/runs" \
      --once \
      "$@"
    ;;
  policy)
    require_output "policy" "$@" || exit 2
    check_ros_sdk
    check_policy_sdk
    exec python3 -m dual_arm_runtime.native_node \
      --project-root "$ROOT" \
      --config "${NATIVE_RUNTIME_CONFIG:-$DEFAULT_POLICY_CONFIG}" \
      --runs-output "$DATA_ROOT/runs" \
      --once \
      "$@"
    ;;
  control)
    check_ros_sdk
    if [[ -n "${NATIVE_RUN_ID:-}" ]]; then
      exec python3 -m dual_arm_runtime.cli --run-id "$NATIVE_RUN_ID" "$@"
    fi
    exec python3 -m dual_arm_runtime.cli "$@"
    ;;
  audit)
    check_ros_sdk
    exec python3 -m dual_arm_runtime.audit "$@"
    ;;
  export)
    check_ros_sdk
    exec python3 "$ROOT/runtime/ros2/scripts/stack_export.py" "$@"
    ;;
  campaign)
    check_ros_sdk
    exec python3 "$ROOT/runtime/ros2/scripts/stack_campaign.py" \
      --project-root "$ROOT" "$@"
    ;;
  evaluate)
    check_ros_sdk
    exec python3 "$ROOT/runtime/ros2/scripts/stack_evaluate.py" \
      --project-root "$ROOT" "$@"
    ;;
  test)
    check_ros_sdk
    exec bash "$ROOT/infrastructure/scripts/test.sh" "$@"
    ;;
  *)
    printf 'runtime run.sh: unknown command %q\n' "$command" >&2
    usage >&2
    exit 2
    ;;
esac

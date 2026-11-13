#!/usr/bin/env bash
# Learning command dispatcher for manual use inside the idle container.
# It remains ROS-free and forwards arguments to the real CLI or service.
set -euo pipefail

SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "${PROJECT_SETUP:-${SHARED_ROOT:-$SCRIPT_ROOT/shared}/common/tools/docker/setup.sh}"

ROOT="${PROJECT_ROOT:-$SCRIPT_ROOT}"
DATA_ROOT="${DATA_ROOT:-/data}"
POLICY_SDK="${POLICY_SDK_ROOT:-/build/generated/policy-sdk}"
ACT_LOG_DIRECTORY="${ACT_LOG_DIRECTORY:-}"

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
usage: learning/lerobot/scripts/run.sh <command> [arguments...]

commands:
  convert         convert accepted exports to a 45/5 LeRobot dataset
  validate        verify dataset split, hashes and source files (verify-data)
  filter          write an auditable static-anchor sampling mask
  train           train ACT smoke/overfit/full
  bundle          export a safetensors-only ACT bundle
  verify-bundle   verify and load a bundle without predicting
  budget          show the append-only training budget
  policy          serve the named ACT Policy API (ROS-free)
  cli             pass arguments directly to dual_arm_learning.cli
  test            run pytest with plugin autoload disabled

Examples:
  learning/lerobot/scripts/run.sh --help
  learning/lerobot/scripts/run.sh validate --scenario panda_stack
  learning/lerobot/scripts/run.sh train --scenario panda_stack --mode smoke
  learning/lerobot/scripts/run.sh convert --scenario panda_stack
  learning/lerobot/scripts/run.sh policy --scenario panda_stack \
    --bundle /data/panda_stack/models/bundles/stack-act
  learning/lerobot/scripts/run.sh policy --config \
    /shared/scenarios/panda_stack/configs/learning/policy_temporal.yaml \
    --bundle-dir /data/panda_stack/models/bundles/stack-act

Every recipe command requires exactly one of --scenario or --config; there is
no implicit panda_stack default.  Policy service logs default to
/data/<scenario-id>/logs/policy-service (`_direct` without a scenario).
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
  convert)
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" convert --progress "$@"
    ;;
  validate)
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" verify-data "$@"
    ;;
  filter)
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" filter-anchors "$@"
    ;;
  train)
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" train --progress "$@"
    ;;
  bundle)
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" bundle "$@"
    ;;
  verify-bundle)
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" verify-bundle "$@"
    ;;
  budget)
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" budget-status "$@"
    ;;
  policy)
    check_policy_sdk
    log_directory="$ACT_LOG_DIRECTORY"
    if [[ -z "$log_directory" ]]; then
      log_directory="$DATA_ROOT/$(scenario_segment "$@")/logs/policy-service"
    fi
    exec python3 -m dual_arm_learning.policy_service_cli \
      --log-directory "$log_directory" \
      --run-id "${ACT_RUN_ID:-policy-$(date -u +%Y%m%dT%H%M%SZ)-$$}" \
      "$@"
    ;;
  cli)
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" "$@"
    ;;
  test)
    exec bash "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/test.sh" "$@"
    ;;
  *)
    printf 'learning run.sh: unknown command %q\n' "$command" >&2
    usage >&2
    exit 2
    ;;
esac

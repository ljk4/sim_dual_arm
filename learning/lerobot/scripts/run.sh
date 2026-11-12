#!/usr/bin/env bash
# Learning command dispatcher for manual use inside the idle container.
# It remains ROS-free and forwards arguments to the real CLI or service.
set -euo pipefail

SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "${PROJECT_SETUP:-${SHARED_ROOT:-$SCRIPT_ROOT/shared}/common/tools/docker/setup.sh}"

ROOT="${PROJECT_ROOT:-$SCRIPT_ROOT}"
DATA_ROOT="${DATA_ROOT:-/data}"
POLICY_SDK="${POLICY_SDK_ROOT:-/build/generated/policy-sdk}"
POLICY_CONFIG="${POLICY_SERVICE_CONFIG:-${ACT_POLICY_CONFIG:-$ROOT/shared/scenarios/panda_stack/configs/learning/policy_temporal.yaml}}"
ACT_LOG_DIRECTORY="${ACT_LOG_DIRECTORY:-$DATA_ROOT/logs/act-policy}"
DATASET_CONFIG="${ACT_DATASET_CONFIG:-$ROOT/shared/scenarios/panda_stack/configs/learning/act_dataset.container.yaml}"
TRAINING_CONFIG="${ACT_TRAINING_CONFIG:-$ROOT/shared/scenarios/panda_stack/configs/learning/act_training.container.yaml}"

has_help=false
for argument in "$@"; do
  if [[ "$argument" == "-h" || "$argument" == "--help" ]]; then
    has_help=true
  fi
done

# A scenario selection must not receive this script's Panda default config.
# An explicit --config also wins over the script default.
has_argument() {
  local option="$1"
  shift
  local argument
  for argument in "$@"; do
    if [[ "$argument" == "$option" || "$argument" == "$option="* ]]; then
      return 0
    fi
  done
  return 1
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
  prepare-policy  verify an ACT bundle and emit Policy binding + service config
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
  learning/lerobot/scripts/run.sh validate
  learning/lerobot/scripts/run.sh train --scenario panda_stack --mode smoke
  learning/lerobot/scripts/run.sh convert --scenario panda_stack
  learning/lerobot/scripts/run.sh prepare-policy --scenario panda_stack \
    --bundle /data/learning/bundles/stack-act --run-id run-30000 \
    --output /data/prepared/run-30000/policy
  learning/lerobot/scripts/run.sh policy --scenario panda_stack \
    --bundle-dir /data/learning/bundles/stack-act
  learning/lerobot/scripts/run.sh prepare-policy --bundle \
    /data/learning/bundles/stack-act --run-id run-30000 \
    --output /data/prepared/run-30000/policy
  learning/lerobot/scripts/run.sh policy --config \
    /data/prepared/run-30000/policy/policy.yaml --log-directory /data/logs/run-30000

--scenario selects recipes through <shared-root>/scenarios/<id>/binding.yaml;
it is mutually exclusive with --config/--template and never receives this
script's Panda default config.  Direct configuration mode is unchanged.
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
  prepare-policy)
    exec python3 "$ROOT/learning/lerobot/scripts/prepare_policy.py" "$@"
    ;;
  convert)
    if has_argument --scenario "$@" || has_argument --config "$@"; then
      exec python3 -m dual_arm_learning.cli --project-root "$ROOT" convert \
        --progress "$@"
    fi
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" convert \
      --config "$DATASET_CONFIG" --progress "$@"
    ;;
  validate)
    if has_argument --scenario "$@" || has_argument --config "$@"; then
      exec python3 -m dual_arm_learning.cli --project-root "$ROOT" verify-data "$@"
    fi
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" verify-data \
      --config "$TRAINING_CONFIG" "$@"
    ;;
  filter)
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" filter-anchors "$@"
    ;;
  train)
    if has_argument --scenario "$@" || has_argument --config "$@"; then
      exec python3 -m dual_arm_learning.cli --project-root "$ROOT" train \
        --progress "$@"
    fi
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" train \
      --config "$TRAINING_CONFIG" --progress "$@"
    ;;
  bundle)
    if has_argument --scenario "$@" || has_argument --config "$@"; then
      exec python3 -m dual_arm_learning.cli --project-root "$ROOT" bundle "$@"
    fi
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" bundle \
      --config "$TRAINING_CONFIG" "$@"
    ;;
  verify-bundle)
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" verify-bundle "$@"
    ;;
  budget)
    if has_argument --scenario "$@" || has_argument --config "$@"; then
      exec python3 -m dual_arm_learning.cli --project-root "$ROOT" budget-status "$@"
    fi
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" budget-status \
      --config "$TRAINING_CONFIG" "$@"
    ;;
  policy)
    check_policy_sdk
    if has_argument --scenario "$@" || has_argument --config "$@"; then
      exec python3 -m dual_arm_learning.policy_service_cli \
        --log-directory "$ACT_LOG_DIRECTORY" \
        --run-id "${ACT_RUN_ID:-policy-$(date -u +%Y%m%dT%H%M%SZ)-$$}" \
        "$@"
    fi
    exec python3 -m dual_arm_learning.policy_service_cli \
      --config "$POLICY_CONFIG" \
      --log-directory "$ACT_LOG_DIRECTORY" \
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

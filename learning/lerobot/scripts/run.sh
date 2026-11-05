#!/usr/bin/env bash
# Learning command dispatcher for manual use inside the idle container.
# It remains ROS-free and forwards arguments to the real CLI or service.
set -euo pipefail

SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$SCRIPT_ROOT/infrastructure/docker/setup.sh"

ROOT="${PROJECT_ROOT:-$SCRIPT_ROOT}"
DATA_ROOT="${DATA_ROOT:-/data}"
POLICY_SDK="${POLICY_SDK_ROOT:-/opt/policy-sdk}"
POLICY_CONFIG="${ACT_POLICY_CONFIG:-$ROOT/learning/lerobot/configs/act_policy.yaml}"
ACT_LOG_DIRECTORY="${ACT_LOG_DIRECTORY:-$DATA_ROOT/logs/act-policy}"
DATASET_CONFIG="${ACT_DATASET_CONFIG:-$ROOT/learning/lerobot/configs/act_dataset.container.yaml}"
TRAINING_CONFIG="${ACT_TRAINING_CONFIG:-$ROOT/learning/lerobot/configs/act_training.container.yaml}"

has_help=false
for argument in "$@"; do
  if [[ "$argument" == "-h" || "$argument" == "--help" ]]; then
    has_help=true
  fi
done

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
  learning/lerobot/scripts/run.sh prepare-policy --bundle \
    /data/learning/bundles/best --run-id run-30000 \
    --output /data/prepared/run-30000/policy
  learning/lerobot/scripts/run.sh policy --config \
    /data/prepared/run-30000/policy/policy.yaml --log-directory /data/logs/run-30000
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
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" convert \
      --config "$DATASET_CONFIG" "$@"
    ;;
  validate)
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" verify-data \
      --config "$TRAINING_CONFIG" "$@"
    ;;
  filter)
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" filter-anchors "$@"
    ;;
  train)
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" train \
      --config "$TRAINING_CONFIG" "$@"
    ;;
  bundle)
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" bundle \
      --config "$TRAINING_CONFIG" "$@"
    ;;
  verify-bundle)
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" verify-bundle "$@"
    ;;
  budget)
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" budget-status \
      --config "$TRAINING_CONFIG" "$@"
    ;;
  policy)
    check_policy_sdk
    exec python3 -m dual_arm_learning.act_service \
      --config "$POLICY_CONFIG" \
      --log-directory "$ACT_LOG_DIRECTORY" \
      --run-id "${ACT_RUN_ID:-policy-$(date -u +%Y%m%dT%H%M%SZ)-$$}" \
      "$@"
    ;;
  cli)
    exec python3 -m dual_arm_learning.cli --project-root "$ROOT" "$@"
    ;;
  test)
    exec bash "$ROOT/infrastructure/scripts/test.sh" "$@"
    ;;
  *)
    printf 'learning run.sh: unknown command %q\n' "$command" >&2
    usage >&2
    exit 2
    ;;
esac

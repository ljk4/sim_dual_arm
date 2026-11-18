#!/usr/bin/env bash
# Run the MuJoCo Robot API backend (robot_backend_sim.common.ros.node) inside the simulation container.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-$ROOT/../../shared/common/tools/docker/setup.sh}"

DATA_ROOT="${DATA_ROOT:-/data}"
ROS_SDK="${ROS_SDK_ROOT:-/opt/ros-dev}"

has_help=false
for argument in "$@"; do
  if [[ "$argument" == "-h" || "$argument" == "--help" ]]; then
    has_help=true
  fi
done

target_segment() {
  local robot="" task="" previous="" argument
  for argument in "$@"; do
    if [[ "$previous" == "--robot" && "$argument" != --* ]]; then
      robot="$argument"
    elif [[ "$previous" == "--task" && "$argument" != --* ]]; then
      task="$argument"
    fi
    case "$argument" in
      --robot=*) robot="${argument#*=}" ;;
      --task=*) task="${argument#*=}" ;;
    esac
    previous="$argument"
  done
  if [[ -n "$robot" && -n "$task" ]]; then
    printf '%s/%s\n' "$robot" "$task"
  else
    printf '_direct\n'
  fi
}

if [[ "$has_help" != "true" ]]; then
  python3 "${SHARED_ROOT:-/shared}/common/tools/scripts/dev_workspace.py" check \
    --project-root / \
    --build-root "$ROS_SDK" 1>&2
fi

exec python3 -m robot_backend_sim.common.ros.node \
  --run-root "$DATA_ROOT/$(target_segment "$@")/logs/backend" \
  "$@"

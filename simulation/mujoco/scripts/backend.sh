#!/usr/bin/env bash
# Run the MuJoCo Robot API backend (robot_backend_sim.ros.node) inside the simulation container.
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

if [[ "$has_help" != "true" ]]; then
  python3 "${SHARED_ROOT:-/shared}/common/tools/scripts/dev_workspace.py" check \
    --project-root / \
    --build-root "$ROS_SDK" 1>&2
fi

exec python3 -m robot_backend_sim.ros.node \
  --run-root "$DATA_ROOT/$(scenario_segment "$@")/logs/backend" \
  "$@"

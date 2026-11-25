#!/usr/bin/env bash
# Start the MuJoCo policy backend for a robot/task inside the simulation container.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-$ROOT/../../shared/common/tools/docker/setup.sh}"

ROS_SDK="${ROS_SDK_ROOT:-/opt/ros-dev}"

want_help=false
for arg in "$@"; do
  if [[ "$arg" == "-h" || "$arg" == "--help" || "$arg" == "help" ]]; then
    want_help=true
    break
  fi
done

if [[ "$want_help" == "true" ]]; then
  exec python3 -m robot_backend_sim.common.ros.node --inference-entry --help
fi

python3 "${SHARED_ROOT:-/shared}/common/tools/scripts/dev_workspace.py" check \
  --project-root / \
  --build-root "$ROS_SDK" >/dev/null

exec python3 -m robot_backend_sim.common.ros.node \
  --inference-entry \
  "$@"

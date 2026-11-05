#!/usr/bin/env bash
# Simulation command dispatcher for manual use inside an existing container.
# It selects the real CLI and forwards arguments unchanged.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-$ROOT/../../infrastructure/docker/setup.sh}"

REPORT_ROOT="${DATA_ROOT:-/data}/reports/sim-check"
ROS_SDK="${ROS_SDK_ROOT:-/opt/ros-dev}"

has_help=false
for argument in "$@"; do
  if [[ "$argument" == "-h" || "$argument" == "--help" ]]; then
    has_help=true
  fi
done

check_ros_sdk() {
  if [[ "$has_help" == "true" ]]; then
    return 0
  fi
  python3 /config/check_ros_sdk.py check \
    --project-root / \
    --build-root "$ROS_SDK" 1>&2
}

usage() {
  cat <<'EOF'
usage: scripts/run.sh <command> [arguments...]

Run this inside the simulation container, for example after
docker compose exec simulation bash from simulation/mujoco/docker.

commands:
  backend       run the MuJoCo Robot API backend (robot_backend_sim.node)
  viewer        open the live MuJoCo GUI for a running backend
  environment   probe the MuJoCo/EGL/camera environment
  scene         check three-camera scene visibility
  kinematics    check key poses and kinematics clearance
  prepare       write backend.yaml + run_manifest.json from a template/seed

Examples:
  scripts/run.sh backend --help
  scripts/run.sh environment --run-id env-001
  scripts/run.sh prepare --template \
    configs/backend/backend_stack_inference.yaml --seed 42 \
    --run-id prep-001 --output /data/prepared/prep-001
  scripts/run.sh backend \
    --config /data/prepared/prep-001/backend.yaml --run-id backend-001
  scripts/run.sh viewer \
    --config /data/prepared/prep-001/backend.yaml

`backend` requires an explicit --config and checks the ROS SDK first.
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
  backend)
    check_ros_sdk
    exec python3 -m robot_backend_sim.node \
      --run-root "${DATA_ROOT:-/data}/runs/backend" \
      "$@"
    ;;
  viewer)
    config=""
    viewer_args=()
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
        --state-topic)
          viewer_args+=(--state-topic "${2:-}")
          shift 2
          ;;
        --state-topic=*)
          viewer_args+=(--state-topic "${1#*=}")
          shift
          ;;
        --poll-hz)
          viewer_args+=(--poll-hz "${2:-}")
          shift 2
          ;;
        --poll-hz=*)
          viewer_args+=(--poll-hz "${1#*=}")
          shift
          ;;
        -h|--help)
          printf '%s\n' \
            'usage: scripts/run.sh viewer --config <backend.yaml> [--state-topic <topic>] [--poll-hz <hz>]'
          exit 0
          ;;
        *)
          printf 'viewer: unknown argument %q\n' "$1" >&2
          exit 2
          ;;
      esac
    done
    if [[ -z "$config" ]]; then
      printf 'viewer: --config <backend.yaml> is required\n' >&2
      exit 2
    fi
    exec env MUJOCO_GL=glfw PYOPENGL_PLATFORM=glx \
      python3 -m robot_backend_sim.viewer --config "$config" "${viewer_args[@]}"
    ;;
  environment)
    exec python3 -m robot_backend_sim.probe_environment \
      --report-root "$REPORT_ROOT" \
      "$@"
    ;;
  scene)
    exec python3 -m robot_backend_sim.check_scene \
      --scene-config "$ROOT/configs/scene/panda_stack.yaml" \
      --check-config "$ROOT/configs/checks/scene_check.yaml" \
      --report-root "$REPORT_ROOT" \
      "$@"
    ;;
  kinematics)
    exec python3 -m robot_backend_sim.check_kinematics \
      --scene-config "$ROOT/configs/scene/panda_stack.yaml" \
      --check-config "$ROOT/configs/checks/scene_check.yaml" \
      --kinematics-config "$ROOT/configs/checks/kinematics_check.yaml" \
      --report-root "$REPORT_ROOT" \
      "$@"
    ;;
  prepare)
    exec python3 "$ROOT/scripts/prepare_backend.py" "$@"
    ;;
  *)
    printf 'simulation run.sh: unknown command %q\n' "$command" >&2
    usage >&2
    exit 2
    ;;
esac

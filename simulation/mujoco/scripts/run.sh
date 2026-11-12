#!/usr/bin/env bash
# Simulation command dispatcher for manual use inside an existing container.
# It selects the real CLI and forwards arguments unchanged.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-$ROOT/../../shared/common/tools/docker/setup.sh}"

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
  python3 "${SHARED_ROOT:-/shared}/common/tools/scripts/dev_workspace.py" check \
    --project-root / \
    --build-root "$ROS_SDK" 1>&2
}

usage() {
  cat <<'EOF'
usage: scripts/run.sh <command> [arguments...]

Run this inside the simulation container, for example after
docker compose exec simulation bash from simulation/mujoco/docker.

commands:
  backend       run the MuJoCo Robot API backend (robot_backend_sim.ros.node)
  viewer        open the live MuJoCo GUI for a running backend
  environment   probe the MuJoCo/EGL/camera environment
  scene         check three-camera scene visibility
  kinematics    check key poses and kinematics clearance
  prepare       write backend.yaml + run_manifest.json from a recipe/seed
  test          run pytest with plugin autoload disabled

Examples:
  scripts/run.sh backend --help
  scripts/run.sh environment --run-id env-001
  scripts/run.sh prepare --template \
    /shared/scenarios/panda_stack/configs/simulation/backend/backend_stack_act_chunked.yaml \
    --seed 42 --run-id prep-001 --output /data/prepared/prep-001
  scripts/run.sh prepare --scenario panda_stack --role policy \
    --bundle /data/learning/bundles/<verified-bundle> \
    --seed 42 --run-id prep-002 --output /data/prepared/prep-002
  scripts/run.sh backend \
    --config /data/prepared/prep-001/backend.yaml --run-id backend-001
  scripts/run.sh viewer \
    --config /data/prepared/prep-001/backend.yaml

`backend` requires an explicit --config and checks the ROS SDK first.
`prepare --scenario` selects a recipe through the scenario binding; direct
`--template` mode is unchanged.  Run entries consume prepared configurations.
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
    exec python3 -m robot_backend_sim.ros.node \
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
      python3 -m robot_backend_sim.render.viewer --config "$config" "${viewer_args[@]}"
    ;;
  environment)
    exec python3 -m robot_backend_sim.tools.probe_environment \
      --report-root "$REPORT_ROOT" \
      "$@"
    ;;
  scene)
    exec python3 -m robot_backend_sim.tools.check_scene \
      --scene-config "${SHARED_ROOT:-/shared}/scenarios/panda_stack/configs/simulation/scene/panda_stack.yaml" \
      --check-config "${SHARED_ROOT:-/shared}/scenarios/panda_stack/configs/simulation/checks/scene_check.yaml" \
      --report-root "$REPORT_ROOT" \
      "$@"
    ;;
  kinematics)
    exec python3 -m robot_backend_sim.tools.check_kinematics \
      --scene-config "${SHARED_ROOT:-/shared}/scenarios/panda_stack/configs/simulation/scene/panda_stack.yaml" \
      --check-config "${SHARED_ROOT:-/shared}/scenarios/panda_stack/configs/simulation/checks/scene_check.yaml" \
      --kinematics-config "${SHARED_ROOT:-/shared}/scenarios/panda_stack/configs/simulation/checks/kinematics_check.yaml" \
      --report-root "$REPORT_ROOT" \
      "$@"
    ;;
  prepare)
    exec python3 "$ROOT/scripts/prepare_backend.py" "$@"
    ;;
  test)
    exec bash "${SHARED_ROOT:-/shared}/common/tools/scripts/test.sh" "$@"
    ;;
  *)
    printf 'simulation run.sh: unknown command %q\n' "$command" >&2
    usage >&2
    exit 2
    ;;
esac

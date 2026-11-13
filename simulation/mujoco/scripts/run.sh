#!/usr/bin/env bash
# Simulation command dispatcher for manual use inside an existing container.
# It selects the real CLI and forwards arguments unchanged.
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
  marvin        compile, step, measure and render the Marvin handover scene
  kinematics    check key poses and kinematics clearance
  handover      build and check the Marvin handover candidate paths
  test          run pytest with plugin autoload disabled

Examples:
  scripts/run.sh backend --help
  scripts/run.sh environment --run-id env-001
  scripts/run.sh backend --scenario panda_stack --role policy \
    --bundle /data/panda_stack/models/bundles/<verified-bundle> \
    --run-id policy-001 --set run.realtime_speed=0.4
  scripts/run.sh viewer --scenario panda_stack --role policy
  scripts/run.sh backend --config /shared/scenarios/panda_stack/configs/simulation/backend/backend_stack_teacher.yaml
  scripts/run.sh viewer --config /shared/scenarios/panda_stack/configs/simulation/backend/backend_stack_teacher.yaml

`backend` accepts exactly one of --scenario or --config.  A scenario is
resolved in memory, expanded and written as <run-root>/<run-id>/backend.yaml
before the simulation starts.  Backend runs default to
/data/<scenario-id>/logs/backend (`_direct` without a scenario); diagnostic
reports default to /data/<scenario-id>/diagnostics/<tool>.
`--set dot.path=value` overrides existing keys.
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
      --run-root "$DATA_ROOT/$(scenario_segment "$@")/logs/backend" \
      "$@"
    ;;
  viewer)
    exec env MUJOCO_GL=glfw PYOPENGL_PLATFORM=glx \
      python3 -m robot_backend_sim.render.viewer "$@"
    ;;
  environment)
    exec python3 -m robot_backend_sim.tools.probe_environment \
      --report-root "$DATA_ROOT/_system/diagnostics/environment" \
      "$@"
    ;;
  scene)
    exec python3 -m robot_backend_sim.tools.check_scene \
      --scene-config "${SHARED_ROOT:-/shared}/scenarios/panda_stack/configs/simulation/scene/panda_stack.yaml" \
      --check-config "${SHARED_ROOT:-/shared}/scenarios/panda_stack/configs/simulation/checks/scene_check.yaml" \
      --report-root "$DATA_ROOT/panda_stack/diagnostics/scene" \
      "$@"
    ;;
  marvin)
    exec python3 -m robot_backend_sim.tools.check_marvin_scene \
      --scene-config "${SHARED_ROOT:-/shared}/scenarios/marvin_handover/configs/simulation/scene/marvin_handover.yaml" \
      --profile "${SHARED_ROOT:-/shared}/scenarios/marvin_handover/profiles/robots/marvin_handover_v1/profile.json" \
      --schemas "${SHARED_ROOT:-/shared}/common/interfaces/schemas" \
      --report-root "$DATA_ROOT/marvin_handover/diagnostics/marvin" \
      "$@"
    ;;
  kinematics)
    exec python3 -m robot_backend_sim.tools.check_kinematics \
      --scene-config "${SHARED_ROOT:-/shared}/scenarios/panda_stack/configs/simulation/scene/panda_stack.yaml" \
      --check-config "${SHARED_ROOT:-/shared}/scenarios/panda_stack/configs/simulation/checks/scene_check.yaml" \
      --kinematics-config "${SHARED_ROOT:-/shared}/scenarios/panda_stack/configs/simulation/checks/kinematics_check.yaml" \
      --report-root "$DATA_ROOT/panda_stack/diagnostics/kinematics" \
      "$@"
    ;;
  handover)
    exec python3 -m robot_backend_sim.tools.check_handover_paths \
      --scene-config "${SHARED_ROOT:-/shared}/scenarios/marvin_handover/configs/simulation/scene/marvin_handover.yaml" \
      --planning-config "${SHARED_ROOT:-/shared}/scenarios/marvin_handover/configs/simulation/planning/handover_paths.yaml" \
      --profile "${SHARED_ROOT:-/shared}/scenarios/marvin_handover/profiles/robots/marvin_handover_v1/profile.json" \
      --schemas "${SHARED_ROOT:-/shared}/common/interfaces/schemas" \
      --report-root "$DATA_ROOT/marvin_handover/diagnostics/handover" \
      "$@"
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

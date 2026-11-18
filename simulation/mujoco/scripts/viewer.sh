#!/usr/bin/env bash
# Open the live MuJoCo GUI viewer for a running backend inside the simulation container.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-$ROOT/../../shared/common/tools/docker/setup.sh}"

exec env MUJOCO_GL=glfw PYOPENGL_PLATFORM=glx \
  python3 -m robot_backend_sim.common.render.viewer "$@"

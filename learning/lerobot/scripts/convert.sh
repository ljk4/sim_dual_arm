#!/usr/bin/env bash
# Convert accepted exports into a 45/5 LeRobot dataset inside the learning container.
set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-${SHARED_ROOT:-$APP_ROOT/../../shared}/common/tools/docker/setup.sh}"

ROOT="${PROJECT_ROOT:-$APP_ROOT}"

exec python3 -m dual_arm_learning.cli --project-root "$ROOT" convert --progress "$@"

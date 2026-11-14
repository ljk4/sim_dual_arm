#!/usr/bin/env bash
# Export a safetensors-only ACT bundle inside the learning container.
set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-${SHARED_ROOT:-$APP_ROOT/../../shared}/common/tools/docker/setup.sh}"

ROOT="${PROJECT_ROOT:-$APP_ROOT}"

exec python3 -m dual_arm_learning.cli --project-root "$ROOT" bundle --progress "$@"

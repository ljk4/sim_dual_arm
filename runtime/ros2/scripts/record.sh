#!/usr/bin/env bash
# Standalone teacher recording entry inside the runtime container.
set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-${SHARED_ROOT:-$APP_ROOT/../../shared}/common/tools/docker/setup.sh}"

exec python3 -m dual_arm_runtime.offline.record \
  --project-root "${PROJECT_ROOT:-$APP_ROOT}" "$@"

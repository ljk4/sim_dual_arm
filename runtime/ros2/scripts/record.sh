#!/usr/bin/env bash
# Standalone teacher recording entry inside the runtime container.
set -euo pipefail

SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "${PROJECT_SETUP:-${SHARED_ROOT:-$SCRIPT_ROOT/shared}/common/tools/docker/setup.sh}"

exec python3 -m dual_arm_runtime.offline.record \
  --project-root "${PROJECT_ROOT:-$SCRIPT_ROOT}" "$@"

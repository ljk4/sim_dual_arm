#!/usr/bin/env bash
# Standalone teacher recording entry inside the runtime container.
set -euo pipefail

SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$SCRIPT_ROOT/infrastructure/docker/setup.sh"

exec python3 -m dual_arm_runtime.offline.record \
  --project-root "${PROJECT_ROOT:-$SCRIPT_ROOT}" "$@"

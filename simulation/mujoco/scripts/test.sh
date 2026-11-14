#!/usr/bin/env bash
# Run pytest inside the simulation container.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-$ROOT/../../shared/common/tools/docker/setup.sh}"

exec bash "${SHARED_ROOT:-/shared}/common/tools/scripts/test.sh" "$@"

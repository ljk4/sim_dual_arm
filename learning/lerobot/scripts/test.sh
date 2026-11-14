#!/usr/bin/env bash
# Run pytest inside the learning container.
set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${PROJECT_SETUP:-${SHARED_ROOT:-$APP_ROOT/../../shared}/common/tools/docker/setup.sh}"

ROOT="${PROJECT_ROOT:-$APP_ROOT}"
exec bash "${SHARED_ROOT:-$ROOT/shared}/common/tools/scripts/test.sh" "$@"

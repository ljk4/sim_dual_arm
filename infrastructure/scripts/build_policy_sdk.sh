#!/usr/bin/env bash
# Generate the Policy SDK with the container's Python and pinned tools.
set -euo pipefail
SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$SCRIPT_ROOT/infrastructure/docker/setup.sh"

ROOT="${PROJECT_ROOT:-$SCRIPT_ROOT}"
BUILD_ROOT="${BUILD_ROOT:-/build}"
SDK_ROOT="${POLICY_SDK_BUILD_ROOT:-$BUILD_ROOT/generated/policy-sdk}"
python3 "$ROOT/infrastructure/scripts/policy_sdk.py" build \
  --project-root "$ROOT" \
  --sdk-root "$SDK_ROOT" \
  "$@"

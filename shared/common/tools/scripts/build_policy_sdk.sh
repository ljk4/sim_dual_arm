#!/usr/bin/env bash
# Generate the Policy SDK with the container's Python and pinned tools.
set -euo pipefail
SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
SHARED_ROOT="${SHARED_ROOT:-$SCRIPT_ROOT/shared}"
source "$SHARED_ROOT/common/tools/docker/setup.sh"

ROOT="${PROJECT_ROOT:-$SCRIPT_ROOT}"
BUILD_ROOT="${BUILD_ROOT:-/build}"
SDK_ROOT="${POLICY_SDK_BUILD_ROOT:-$BUILD_ROOT/generated/policy-sdk}"
python3 "$SHARED_ROOT/common/tools/scripts/policy_sdk.py" build \
  --project-root "$ROOT" \
  --sdk-root "$SDK_ROOT" \
  "$@"

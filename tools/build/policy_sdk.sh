#!/usr/bin/env bash
# Generate/verify an owned host SDK using container dependencies only.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DOCKER="${DEV_DOCKER:-docker}"
IMAGE="${POLICY_BUILD_IMAGE:-${DEV_IMAGE:-sim-dual-arm/dev-ros:stage2}}"
SDK_ROOT="${POLICY_SDK_ROOT:-$ROOT/.build/generated/policy-sdk}"
if [[ "$SDK_ROOT" != /* ]]; then
  SDK_ROOT="$ROOT/$SDK_ROOT"
fi
mkdir -p "$SDK_ROOT"

"$DOCKER" run --rm --pull=never --network none \
  --user "$(id -u):$(id -g)" \
  --env PYTHONDONTWRITEBYTECODE=1 \
  --mount "type=bind,source=$ROOT,target=/workspace,readonly" \
  --mount "type=bind,source=$SDK_ROOT,target=/policy-sdk" \
  --entrypoint python3 "$IMAGE" \
  /workspace/tools/build/policy_sdk.py build \
  --project-root /workspace --sdk-root /policy-sdk

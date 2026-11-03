#!/usr/bin/env bash
# Validate mounted source/SDK before starting the requested Policy process.
set -euo pipefail

ROOT="${POLICY_REPO_ROOT:-/workspace}"
SDK_ROOT="${POLICY_SDK_ROOT:-/opt/policy-sdk}"
export PYTHONPATH="$SDK_ROOT${PYTHONPATH:+:$PYTHONPATH}"
python3 "$ROOT/tools/build/policy_sdk.py" check \
  --project-root "$ROOT" --sdk-root "$SDK_ROOT" 1>&2
exec "$@"

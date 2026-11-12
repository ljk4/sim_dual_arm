#!/usr/bin/env bash
# Shared test entry; each environment supplies its own dependency set.
set -euo pipefail

SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
SHARED_ROOT="${SHARED_ROOT:-$SCRIPT_ROOT/shared}"
ROOT="${PROJECT_ROOT:-$SCRIPT_ROOT}"
source "$SHARED_ROOT/common/tools/docker/setup.sh"

# ROS launch_testing otherwise imports unselected sibling modules, allowing an
# unrelated optional dependency to skip an entire directory during collection.
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$ROOT/tests/fakes/policy${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m pytest -p no:cacheprovider "$@"

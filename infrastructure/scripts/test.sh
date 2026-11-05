#!/usr/bin/env bash
# Shared test entry; each environment supplies its own dependency set.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$ROOT/infrastructure/docker/setup.sh"

# ROS launch_testing otherwise imports unselected sibling modules, allowing an
# unrelated optional dependency to skip an entire directory during collection.
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$ROOT/tests/doubles/policy:$ROOT/tests/doubles/ros/mock_robot_backend${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m pytest -p no:cacheprovider "$@"

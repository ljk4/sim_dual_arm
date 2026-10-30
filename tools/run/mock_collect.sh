#!/usr/bin/env bash
# Thin entrypoint; the Python runner exports data and cleans only its own resources.
set -euo pipefail
cd "$(dirname "$0")/../.."
exec python3 tools/run/mock_collect.py "$@"

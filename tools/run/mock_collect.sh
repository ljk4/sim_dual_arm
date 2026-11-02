#!/usr/bin/env bash
# Thin entrypoint for the Compose collection supervisor.
set -euo pipefail
cd "$(dirname "$0")/../.."
exec python3 tools/run/mock_collect.py "$@"

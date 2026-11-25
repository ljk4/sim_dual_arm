#!/usr/bin/env bash
# Start the MuJoCo teacher backend for a robot/backend/task inside the mujoco container.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for arg in "$@"; do
  if [[ "$arg" == "-h" || "$arg" == "--help" || "$arg" == "help" ]]; then
    exec "$ROOT/scripts/backend.sh" --help
  fi
done
exec "$ROOT/scripts/backend.sh" --role teacher --run-id teacher-backend "$@"

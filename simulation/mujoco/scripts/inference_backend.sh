#!/usr/bin/env bash
# Start the MuJoCo policy backend for a robot/task inside the simulation container.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for arg in "$@"; do
  if [[ "$arg" == "-h" || "$arg" == "--help" || "$arg" == "help" ]]; then
    exec "$ROOT/scripts/backend.sh" --help
  fi
done
exec "$ROOT/scripts/backend.sh" --role policy --seed 20000 --run-id policy-backend "$@"

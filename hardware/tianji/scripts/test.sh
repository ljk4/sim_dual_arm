#!/usr/bin/env bash
set -euo pipefail

hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
project_root="${PROJECT_ROOT:-$(cd "$hardware_root/../.." && pwd)}"
export PROJECT_ROOT="$project_root"
export PROJECT_SOURCE_PATHS="$hardware_root/src${PROJECT_SOURCE_PATHS:+:$PROJECT_SOURCE_PATHS}"
export PYTHONPATH="$hardware_root/src${PYTHONPATH:+:$PYTHONPATH}"
cd "$hardware_root"
if [[ $# -eq 0 ]]; then
  set -- "$hardware_root/tests"
fi
exec bash "${SHARED_ROOT:-$project_root/shared}/common/tools/scripts/test.sh" "$@"

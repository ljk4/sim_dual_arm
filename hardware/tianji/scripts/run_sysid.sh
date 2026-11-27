#!/usr/bin/env bash
set -euo pipefail

hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
project_root="${PROJECT_ROOT:-$(cd "$hardware_root/../.." && pwd)}"
source "${PROJECT_SETUP:-$project_root/shared/common/tools/docker/setup.sh}"
export PYTHONPATH="$hardware_root/src${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m robot_backend_hardware.tianji.calibration.sysid "$@"

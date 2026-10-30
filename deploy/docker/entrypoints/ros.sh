#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
if [[ -f /opt/project/install/setup.bash ]]; then
  source /opt/project/install/setup.bash
fi
set -u
exec "$@"
